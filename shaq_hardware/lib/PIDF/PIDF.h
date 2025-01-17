#ifndef PIDF_H
#define PIDF_H

#include <Arduino.h>

class PIDF {
  private:
    unsigned long long LastTime;
  public:
    float Kp, Ki, Kd, Kf, Setpoint, Error, LastError;
    float Dt, Integral, i_min, i_max, min_val, max_val;
    float compute(float, float);
    // float Calculate(float);
    static byte SigNum(float);
    PIDF(float, float, float, float, float, float, float, float);
    void setPIDF(float, float, float, float);
    void reset();
};

#endif