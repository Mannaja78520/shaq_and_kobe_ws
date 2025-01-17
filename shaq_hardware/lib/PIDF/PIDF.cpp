#include "PIDF.h"

PIDF::PIDF(float i_min, float i_max, float min_val, float max_val, float Kp, float Ki, float Kd, float Kf) {
  this->setPIDF(Kp, Ki, Kd, Kf);
  this->i_min = i_min;
  this->i_max = i_max;
  this->min_val = min_val;
  this->max_val = max_val;
  Setpoint = Dt = Error = Integral = LastTime = LastError = 0;
}

void PIDF::setPIDF(float Kp, float Ki, float Kd, float Kf) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->Kf = Kf;
}

void PIDF::reset(){
  this->LastTime = 0;
  this->Integral = 0;
  this->LastError = 0;
  this->LastTime = 0;
}

float PIDF::compute(float setpoint, float measure){
  Setpoint = setpoint;
  unsigned long long CurrentTime = millis();
  Dt = (CurrentTime - LastTime) / 1000.0;
  if (Dt <  1E-6) return;
  // Serial.println(Dt);
  Error = setpoint - measure;
  Integral += Error * Dt;
  // Integral += Error;
  if(i_min != -1 && i_max != -1) Integral = constrain(Integral, this->i_min, this->i_max);
  float Derivative = (Error - LastError) / Dt;
  // float Derivative = Error - LastError;
  if (Setpoint == 0 && Error == 0) {
    Integral = 0;
    Derivative = 0;
  }
  LastError = Error;
  LastTime = CurrentTime;
  if (setpoint == 0 && Kf > 0) return 0;
  return constrain(Kp * Error + Ki * Integral + Kd * Derivative + Kf * SigNum(Error), min_val, max_val);
}

byte PIDF::SigNum(float number) {
    return (byte) (number == 0 ? 0 : (number < 0 ? -1 : 1));
}