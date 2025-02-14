// PID_v2.cpp - Implementation with Auto-Tuning
#include "PID_v2.h"

PID_v2::PID_v2(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int POn, int ControllerDirection) {
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;
    SetOutputLimits(0, 1000);
    SampleTime = 250;
    SetControllerDirection(ControllerDirection);
    SetTunings(Kp, Ki, Kd, POn);
    lastTime = millis() - SampleTime;
}

PID_v2::PID_v2(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd, int ControllerDirection)
    : PID_v2(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection) {}

bool PID_v2::Compute() {
    if (!inAuto) return false;
    unsigned long now = millis();
    unsigned long timeChange = now - lastTime;

    if (timeChange >= SampleTime) {
        double input = *myInput;
        double error = *mySetpoint - input;
        double dInput = (input - lastInput);
        outputSum += ki * error;
        Clamp(outputSum);
        double output = pOnE ? kp * error : 0;
        if (!pOnE) output += kp * dInput;
        output -= kd * dInput;
        Clamp(output);
        *myOutput = output;
        lastInput = input;
        lastTime = now;
        return true;
    }
    return false;
}

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

void PID_v2::SetMode(int Mode) {
    bool newAuto = (Mode == AUTOMATIC);
    if (newAuto && !inAuto) {
        Initialize();
    }
    inAuto = newAuto;
}

void PID_v2::Initialize() {
    outputSum = *myOutput;
    lastInput = *myInput;
    Clamp(outputSum);
}

void PID_v2::Clamp(double &output) {
    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;
}

void PID_v2::AutoTune(double relayAmplitude, int cycles) {
    this->relayAmplitude = relayAmplitude;
    maxCycles = cycles;
    tuningCycle = 0;
    tuningActive = true;
    StartTuning();
}

void PID_v2::StartTuning() {
    peak1 = peak2 = 0;
    Ku = Pu = 0;
}

bool PID_v2::IsTuningComplete() {
    return !tuningActive;
}

void PID_v2::SetControllerDirection(int Direction) {
    if (inAuto && Direction != controllerDirection) {
        kp = -kp;
        ki = -ki;
        kd = -kd;
    }
    controllerDirection = Direction;
}
