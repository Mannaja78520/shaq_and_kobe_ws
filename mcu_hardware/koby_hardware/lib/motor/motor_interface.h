#ifndef MOTOR_INTERFACE
#define MOTOR_INTERFACE

class MotorInterface
{
    bool invert_, brakemotor_;

protected:
    virtual void forward(int pwm) = 0;
    virtual void reverse(int pwm) = 0;

public:
    MotorInterface(int invert, int brakemotortype) : invert_(invert), brakemotor_(brakemotortype)
    {
    }

    virtual void brake() = 0;
    virtual void floatmotor() = 0;
    void spin(int pwm)
    {
        if (invert_)
            pwm *= -1;

        if (pwm > 0)
            forward(pwm);
        else if (pwm < 0)
            reverse(pwm);
        else
        {
            if (brakemotor_)
                brake();
            else
                floatmotor();
        }
    }
};

#endif