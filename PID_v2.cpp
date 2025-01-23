#include "PID_v2.h"
#include "Arduino.h"

// Constructor with P, I, D, POn, and Direction
PID_v2::PID_v2(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection) {
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;

    SetOutputLimits(0, 1000);  // Default output limits for PWM range (0-255)
    SampleTime = 250;          // Default sample time is 30 ms (can be changed)
    SetControllerDirection(ControllerDirection);
    SetTunings(Kp, Ki, Kd, POn);
    lastTime = millis() - SampleTime;
}

// Constructor without Proportional on Error (defaults to P_ON_E)
PID_v2::PID_v2(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection)
    : PID_v2::PID_v2(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection) {}

// Compute the PID output value
bool PID_v2::Compute() {
    if (!inAuto) return false;

    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);

    if (timeChange >= SampleTime) {
        // Compute the PID terms
        double input = *myInput;
        double error = *mySetpoint - input;
        double dInput = (input - lastInput);

        // Integral term
        outputSum += ki * error;

        // Anti-windup: Clamp integral term to output limits
        if (outputSum > outMax) outputSum = outMax;
        else if (outputSum < outMin) outputSum = outMin;

        // Proportional term (based on error or measurement)
        double output = pOnE ? kp * error : 0;
        if (!pOnE) output += kp * dInput;

        // Derivative term
        output -= kd * dInput;

        // Final output after clamping
        Clamp(output);

        // Set the output and store the last values for next cycle
        *myOutput = output;
        lastInput = input;
        lastTime = now;
        return true;
    }

    return false;
}

// Set the PID tuning parameters (Kp, Ki, Kd)
void PID_v2::SetTunings(double Kp, double Ki, double Kd, int POn) {
    if (Kp < 0 || Ki < 0 || Kd < 0) return;

    pOn = POn;
    pOnE = (POn == P_ON_E);

    dispKp = Kp;
    dispKi = Ki;
    dispKd = Kd;

    double SampleTimeInSec = (double)SampleTime / 1000;
    kp = Kp;
    ki = Ki * SampleTimeInSec;
    kd = Kd / SampleTimeInSec;

    if (controllerDirection == REVERSE) {
        kp = -kp;
        ki = -ki;
        kd = -kd;
    }
}

// Set sample time (in milliseconds)
void PID_v2::SetSampleTime(int NewSampleTime) {
    if (NewSampleTime > 0) {
        double ratio = (double)NewSampleTime / (double)SampleTime;
        ki *= ratio;
        kd /= ratio;
        SampleTime = (unsigned long)NewSampleTime;
    }
}

// Set the output limits (default is 0-255 for PWM)
void PID_v2::SetOutputLimits(double Min, double Max) {
    if (Min >= Max) return;
    outMin = Min;
    outMax = Max;

    if (inAuto) {
        // Clamp output and sum to the output limits
        if (*myOutput > outMax) *myOutput = outMax;
        else if (*myOutput < outMin) *myOutput = outMin;

        if (outputSum > outMax) outputSum = outMax;
        else if (outputSum < outMin) outputSum = outMin;
    }
}

// Set the control mode (Automatic or Manual)
void PID_v2::SetMode(int Mode) {
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !inAuto) {
        Initialize();
    }
    inAuto = newAuto;
}

// Initialize the controller state
void PID_v2::Initialize() {
    outputSum = *myOutput;
    lastInput = *myInput;

    if (outputSum > outMax) outputSum = outMax;
    else if (outputSum < outMin) outputSum = outMin;
}

// Set the controller direction (Direct or Reverse)
void PID_v2::SetControllerDirection(int Direction) {
    if (inAuto && Direction != controllerDirection) {
        kp = -kp;
        ki = -ki;
        kd = -kd;
    }
    controllerDirection = Direction;
}

// Getter functions for Kp, Ki, Kd, Mode, and Direction
double PID_v2::GetKp() { return dispKp; }
double PID_v2::GetKi() { return dispKi; }
double PID_v2::GetKd() { return dispKd; }
int PID_v2::GetMode() { return inAuto ? AUTOMATIC : MANUAL; }
int PID_v2::GetDirection() { return controllerDirection; }

// Clamping helper function
void PID_v2::Clamp(double &output) {
    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;
}
