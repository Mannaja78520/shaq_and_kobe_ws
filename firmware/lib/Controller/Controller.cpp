#include "Controller.h"

Controller::Controller(float Kp, float Ki, float Kd, float Kf) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->Kf = Kf;
  Setpoint = Dt = Error = Integral = LastTime = LastError = 0;        
}

void Controller::setPIDF(float Kp, float Ki, float Kd, float Kf) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->Kf = Kf;
}

void Controller::reset(){
  this->LastTime = 0;
  this->Integral = 0;
  this->LastError = 0;
  this->LastTime = 0;
}

float Controller::Calculate(float setpoint, float measure){
  Setpoint = setpoint;
  return Calculate(setpoint - measure);
}

float Controller::Calculate(float error){
  unsigned long CurrentTime;
  // Serial.println(error);
  CurrentTime = millis();
  Dt = CurrentTime / 1000.0 - LastTime / 1000.0;
  // Serial.println(Dt);
  LastTime = CurrentTime;
  Error = error;
  Integral += Error * Dt;
  Integral = constrain(Integral, -100, 100);
 if (error == 0) Integral = 0;
  float Derivative = (Error - LastError) / Dt;
  LastError = Error;
  return Kp * Error + Ki * Integral + Kd * Derivative + Kf * SigNum(error);
}

byte Controller::SigNum(float number) {
    return (byte) (number == 0 ? 0 : (number < 0 ? -1 : 1));
}