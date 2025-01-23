#ifndef PID_v2_h
#define PID_v2_h
#define LIBRARY_VERSION    2.0.0

// Include platform-specific headers
#if defined(ESP32) || defined(ESP8266)
    #include "Arduino.h"  // For ESP32 and ESP8266
#elif defined(STM32)
    #include "Arduino.h"  // STM32 uses the same library as Arduino
#else
    #error "Unsupported platform"
#endif

class PID_v2 {

public:
    // Constants for PID modes
    #define AUTOMATIC   1
    #define MANUAL      0
    #define DIRECT      0
    #define REVERSE     1
    #define P_ON_M      0
    #define P_ON_E      1

    // Constructor with PID parameters
    PID_v2(double*, double*, double*, double, double, double, int, int);
    PID_v2(double*, double*, double*, double, double, double, int);

    // Commonly used functions
    void SetMode(int Mode);
    bool Compute();
    void SetOutputLimits(double, double);

    // Functions for adjusting the PID tuning parameters
    void SetTunings(double, double, double);
    void SetTunings(double, double, double, int);

    // Functions for controlling sample time and direction
    void SetControllerDirection(int);
    void SetSampleTime(int);

    // Getters for internal values
    double GetKp();
    double GetKi();
    double GetKd();
    int GetMode();
    int GetDirection();

private:
    void Initialize();
    void Clamp(double &output);

    // Internal variables for PID calculations
    double dispKp, dispKi, dispKd;
    double kp, ki, kd;
    int controllerDirection;
    int pOn;

    // Links to user-provided input, output, and setpoint
    double* myInput;
    double* myOutput;
    double* mySetpoint;

    // Timing and error variables
    unsigned long lastTime;
    double outputSum, lastInput;
    unsigned long SampleTime;
    double outMin, outMax;

    // Internal state for PID calculations
    bool inAuto, pOnE;
};

#endif
