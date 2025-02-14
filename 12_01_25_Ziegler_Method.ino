#include <Adafruit_MAX31865.h>
#include <WiFi.h>
#include <ModbusRTUSlave.h>
#include <PID_v2.h>

#define MAX31865_MISO 19
#define MAX31865_MOSI 23
#define MAX31865_CLK 18
#define MAX31865_CS 5

#define MAX485_DE 32
#define MAX485_RE 33
#define MAX485_RX 16
#define MAX485_TX 17

Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX31865_CS, MAX31865_MOSI, MAX31865_MISO, MAX31865_CLK);

ModbusRTUSlave modbus_slave(Serial1);
const uint8_t slaveID = 1;
const uint32_t baud = 9600;
uint16_t holdingRegisters[6] = {0}; 

const char* ssid = "equichem_5";
const char* password = "equichem@2023";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;  // GMT offset for Asia/Kolkata (UTC+5:30)
const int daylightOffset_sec = 0;

#define R0 100.0
#define Rref 430.0

#define A 3.9083e-3
#define B -5.775e-7
#define C -4.183e-12

#define SSR_PIN 22  

double setpoint = 00.0;  // Setpoint (Initial value)
double lastSetpoint = 00.0;  // Store last setpoint for stability

#define totalReadingsMAX31865 2
double temperatureReadingsMAX31865[totalReadingsMAX31865] = {0};

unsigned long lastReadingTime = 0;
unsigned long lastControlCycleTime = 0;
const long readingInterval = 250;  // Interval between readings (250ms)
const long controlCycleInterval = 2000;  // Control cycle time (5000ms)

float previousEMA = 0.0;
const float alpha = 0.1;  // EMA smoothing factor

// PID control variables
double inputTemperature, output; 

// Ziegler-Nichols Tuning Parameters (Ku and Pu should be determined experimentally)
double Ku = 4.0;  // Ultimate gain (Ku) 
double Pu = 2.0;   // Ultimate period (Pu)

// Ziegler-Nichols formulas for PID
double Kp = 0.5 * Ku;  // Proportional gain
double Ki = 1.0 * Kp / Pu;  // Integral gain
double Kd = Kp * Pu / 8.0;  // Derivative gain

PID_v2 pidController(&inputTemperature, &output, &setpoint, Kp, Ki, Kd, P_ON_E, DIRECT);  // PID controller initialization

void setup() {
  Serial.begin(115200);
  pinMode(SSR_PIN, OUTPUT); 
  digitalWrite(SSR_PIN, LOW);  // SSR is off initially

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to WiFi...");
    delay(1000);  // Still required here for WiFi connection
  }
  Serial.println("WiFi connected.");

  // Initialize NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  for (int i = 0; i < 5; i++) {
    if (getLocalTime(&timeinfo)) {
      Serial.println("Time synchronized");
      break;
    } else {
      Serial.println("Failed to get time, retrying...");
      delay(250);  
    }
  }

  // Initialize MAX31865
  thermo.begin(MAX31865_3WIRE);

  // Initialize PID control
  pidController.SetMode(AUTOMATIC);  // Start PID control
  pidController.SetOutputLimits(0, 255);  // Limit PID output to control SSR (0-255)

  // Initialize Modbus RTU (RS-485)
  pinMode(MAX485_DE, OUTPUT);
  pinMode(MAX485_RE, OUTPUT);
  digitalWrite(MAX485_DE, LOW);  // Set DE low for receiver mode
  digitalWrite(MAX485_RE, HIGH); // Set RE high for receiver mode
  Serial1.begin(baud, SERIAL_8N1, MAX485_RX, MAX485_TX);

  // Configure Modbus slave
  modbus_slave.configureHoldingRegisters(holdingRegisters, 6);  // 6 registers now
  modbus_slave.begin(slaveID, baud, SERIAL_8N1);

  Serial.println("MAX31865 Temperature Sensor Initialized");

  // Create tasks
  xTaskCreatePinnedToCore(readingTEMP_Modbus, "Combined Task", 2048, NULL, 1, NULL, 0);  
  xTaskCreatePinnedToCore(controlSSRTask, "Control SSR", 2048, NULL, 1, NULL, 1);
}

void loop() {
    
}

void readingTEMP_Modbus(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    // Wait for the next interval (250ms)
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(readingInterval));  // Accurate timing with vTaskDelayUntil

    // Temperature reading section
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      unsigned long ms = millis() % 1000;  // Get current milliseconds
      char formattedTime[20];
      snprintf(formattedTime, sizeof(formattedTime), "%02d:%02d:%02d:%03lu", 
               timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, ms);

      // Read from MAX31865 sensor (multiple readings for smoothing)
      for (int i = 0; i < totalReadingsMAX31865; i++) {
        uint16_t adcCode = thermo.readRTD();
        float Rt = (adcCode * Rref) / 32768.0;
        temperatureReadingsMAX31865[i] = Rt;
      }

      // Calculate average temperature
      double tempSumMax31865 = 0;
      for (int i = 0; i < totalReadingsMAX31865; i++) {
        tempSumMax31865 += temperatureReadingsMAX31865[i];
      }
      float averagedRtMax31865 = tempSumMax31865 / totalReadingsMAX31865;
      float temperatureMax31865 = calculateTemperature(averagedRtMax31865);

      // Apply Exponential Moving Average (EMA)
      previousEMA = alpha * temperatureMax31865 + (1 - alpha) * previousEMA;

      // Update Modbus holding registers with the adjusted temperature
      holdingRegisters[1] = (uint16_t)(previousEMA * 10);  // Modbus register holds value in tenths of a degree
      // Update time values in Modbus registers
      holdingRegisters[2] = timeinfo.tm_hour;  
      holdingRegisters[3] = timeinfo.tm_min;   
      holdingRegisters[4] = timeinfo.tm_sec; 
      holdingRegisters[5] = ms;  // Milliseconds

      Serial.print("Temperature: ");
      Serial.print(previousEMA, 2);
      Serial.print("°C,  ");
      Serial.print("Setpoint: ");
      Serial.print(setpoint, 1);  // Print the setpoint with one decimal place
      Serial.println("°C");

      // Transmit Modbus data
      transmitMode();
      modbus_slave.poll();  // Poll Modbus slave for new data
      receiveMode();  // Switch back to receive mode
    } else {
      Serial.println("Failed to obtain time");
    }

    // Modbus update section
    modbus_slave.poll();  // Poll Modbus slave for new data
    
    // Get the setpoint from holding register
    double newSetpoint = holdingRegisters[0] / 10.0;  // Convert from tenths of degrees

    // Update setpoint if it has changed significantly
    if (abs(newSetpoint - lastSetpoint) > 0.1) {
      setpoint = newSetpoint;
      lastSetpoint = setpoint;
    }

    // Clamp setpoint to the defined limits (+35°C to +60°C)
    if (setpoint < 25.0) {
      setpoint = 25.0;
    } else if (setpoint > 60.0) {
      setpoint = 60.0;
    }
  }
}

void controlSSRTask(void *pvParameters) {
  static unsigned long lastControlCycleTime = 0;
  static unsigned long lastSSRUpdateTime = 0;
  static bool isSSROn = false;
  int pwmValue = 0;

  while (1) {
    unsigned long currentMillis = millis();
    unsigned long elapsedControlCycleTime = currentMillis - lastControlCycleTime;
    
    // Check if enough time has passed for a new control cycle
    if (elapsedControlCycleTime >= controlCycleInterval) {
      lastControlCycleTime = currentMillis;  // Update control cycle time

      // Update PID control input with the most recent temperature reading
      inputTemperature = previousEMA;

      // Compute the PID output
      pidController.Compute();

      // Calculate PWM duty cycle based on PID output
      pwmValue = (output / 4.0) * controlCycleInterval;
      pwmValue = constrain(pwmValue, 0, controlCycleInterval);  

      // Print PID output and PWM duty cycle information
      Serial.print("PID Output:  ");
      Serial.print(output, 2);
      Serial.print(", PWM Duty Cycle: ");
      Serial.print(pwmValue);
      Serial.print(" ms (");
      Serial.print((pwmValue / float(controlCycleInterval)) * 100, 2);  // Duty cycle in percentage
      Serial.print("%), ");
    }

    // Now check whether to turn SSR on or off based on the calculated duty cycle
    unsigned long elapsedSSRTime = currentMillis - lastSSRUpdateTime;
    if (elapsedSSRTime < pwmValue) {
      // SSR should be ON during the calculated pwmValue duration
      if (!isSSROn) {
        digitalWrite(SSR_PIN, HIGH);  // Turn heater ON
        isSSROn = true;
      }
    } else {
      // SSR should be OFF after the pwmValue duration has passed
      if (isSSROn) {
        digitalWrite(SSR_PIN, LOW);  // Turn heater OFF
        isSSROn = false;
      }
    }

    // Periodically reset the SSR update time after control cycle interval
    if (elapsedSSRTime >= controlCycleInterval) {
      lastSSRUpdateTime = currentMillis;  // Reset for next cycle
    }
  }
}

// Calculate temperature from RTD resistance
float calculateTemperature(float Rt) {
  float t;
  if (Rt >= R0) {
    t = (-A + sqrt(A * A - 4 * B * (1 - Rt / R0))) / (2 * B);
  } else {
    float tolerance = 0.001;
    int maxIterations = 100;
    int iteration = 0;
    float diff;
    t = Rt;

    do {
      float fValue = R0 * (1 + A * t + B * t * t + C * (t - 100) * t * t * t) - Rt;
      float fDerivative = R0 * (A + 2 * B * t + 3 * C * (t - 100) * t * t + C * t * t * t);
      float nextT = t - fValue / fDerivative;
      diff = abs(nextT - t);
      t = nextT;
      iteration++;
    } while (diff > tolerance && iteration < maxIterations);

    if (iteration == maxIterations) {
      Serial.println("Warning: Temperature calculation failed to converge.");
    }
  }
  return t;
}

void transmitMode() {
  digitalWrite(MAX485_DE, HIGH);  // Enable Driver (Transmit mode)
  digitalWrite(MAX485_RE, HIGH);  // Disable Receiver
}

void receiveMode() {
  digitalWrite(MAX485_DE, LOW);   // Disable Driver (Receive mode)
  digitalWrite(MAX485_RE, LOW);   // Enable Receiver
}
