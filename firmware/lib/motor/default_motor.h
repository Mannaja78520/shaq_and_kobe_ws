#ifndef DEFAULT_MOTOR
#define DEFAULT_MOTOR

#include <Arduino.h>

#include "motor_interface.h"

class PRIK : public MotorInterface
{
private:
    int in_a_pin_;
    int in_b_pin_;
    int pwm_pin_;

protected:
    void forward(int pwm) override
    {
        digitalWrite(in_a_pin_, HIGH);
        digitalWrite(in_b_pin_, LOW);
        analogWrite(pwm_pin_, abs(pwm));
    }

    void reverse(int pwm) override
    {
        digitalWrite(in_a_pin_, LOW);
        digitalWrite(in_b_pin_, HIGH);
        analogWrite(pwm_pin_, abs(pwm));
    }

public:
    PRIK(float pwm_frequency, int pwm_bits, bool invert, bool brakemotortype, int pwm_pin, int in_a_pin, int in_b_pin) : MotorInterface(invert, brakemotortype),
                                                                                                                         in_a_pin_(in_a_pin),
                                                                                                                         in_b_pin_(in_b_pin),
                                                                                                                         pwm_pin_(pwm_pin)
    {
        pinMode(in_a_pin_, OUTPUT);
        pinMode(in_b_pin_, OUTPUT);
        pinMode(pwm_pin_, OUTPUT);

        if (pwm_frequency > 0)
        {
            analogWriteFrequency(pwm_frequency);
        }
        analogWriteResolution(pwm_bits);

        analogWrite(pwm_pin_, 0);
    }

    void brake() override
    {
        digitalWrite(in_a_pin_, HIGH);
        digitalWrite(in_b_pin_, HIGH);
        analogWrite(pwm_pin_, 0);
    }

    void floatmotor() override
    {
        digitalWrite(in_a_pin_, LOW);
        digitalWrite(in_b_pin_, LOW);
        analogWrite(pwm_pin_, 0);
    }
};

#endif
