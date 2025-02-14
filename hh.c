// PID_v2.h - PID Controller with Auto-Tuning
#ifndef PID_v2_h
#define PID_v2_h
#define LIBRARY_VERSION 2.1.0

#include "Arduino.h"

#if defined(ESP32) || defined(ESP8266)
    #include "Arduino.h"
#elif defined(STM32)
    #include "Arduino.h"
#else
    #error "Unsupported platform"
#endif

class PID_v2 {
public:
    #define AUTOMATIC   1
    #define MANUAL      0
    #define DIRECT      0
    #define REVERSE     1
    #define P_ON_M      0
    #define P_ON_E      1

    PID_v2(double*, double*, double*, double, double, double, int, int);
    PID_v2(double*, double*, double*, double, double, double, int);

    void SetMode(int Mode);
    bool Compute();
    void SetOutputLimits(double, double);
    void SetTunings(double, double, double);
    void SetTunings(double, double, double, int);
    void SetControllerDirection(int);
    void SetSampleTime(int);

    double GetKp();
    double GetKi();
    double GetKd();
    int GetMode();
    int GetDirection();

    // Auto-Tuning Feature
    void AutoTune(double relayAmplitude, int cycles);
    bool IsTuningComplete();

private:
    void Initialize();
    void Clamp(double &output);
    void StartTuning();

    double dispKp, dispKi, dispKd;
    double kp, ki, kd;
    int controllerDirection;
    int pOn;

    double* myInput;
    double* myOutput;
    double* mySetpoint;

    unsigned long lastTime;
    double outputSum, lastInput;
    unsigned long SampleTime;
    double outMin, outMax;

    bool inAuto, pOnE;

    // Auto-Tuning Variables
    bool tuningActive;
    double peak1, peak2;
    double Ku, Pu;
    double relayAmplitude;
    int tuningCycle, maxCycles;
};

#endif