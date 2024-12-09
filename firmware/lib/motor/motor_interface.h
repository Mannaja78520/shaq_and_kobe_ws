#ifndef MOTOR_INTERFACE
#define MOTOR_INTERFACE

class MotorInterface
{
private:
    bool invert_;     // ทิศทางกลับด้าน
    bool brakemotor_; // ใช้เบรกหรือปล่อยลอย

protected:
    // ฟังก์ชันควบคุมที่คลาสลูกต้อง override
    virtual void forward(int pwm) = 0;
    virtual void reverse(int pwm) = 0;

public:
    // Constructor พร้อมการตรวจสอบค่าพารามิเตอร์
    MotorInterface(bool invert, bool brakemotor)
        : invert_(invert), brakemotor_(brakemotor) {}

    // ฟังก์ชันเสมือนที่ต้องถูก override ในคลาสลูก
    virtual void brake() = 0;
    virtual void floatmotor() = 0;

    // ฟังก์ชันควบคุมการหมุนของมอเตอร์
    void spin(int pwm)
    {
        if (pwm == 0)
        {
            if (brakemotor_)
                brake();
            else
                floatmotor();
            return;
        }

        if (invert_)
            pwm *= -1;

        if (pwm > 0)
            forward(pwm);
        else
            reverse(pwm);
    }

    // ฟังก์ชันปรับค่าทิศทางแบบ runtime
    void setInvert(bool invert)
    {
        invert_ = invert;
    }

    // ฟังก์ชันปรับประเภทเบรกแบบ runtime
    void setBrakeMode(bool brakemotor)
    {
        brakemotor_ = brakemotor;
    }

    // ฟังก์ชันแสดงสถานะการตั้งค่า (Debugging)
    void debugStatus() const
    {
        Serial.print("Invert: ");
        Serial.println(invert_ ? "Enabled" : "Disabled");
        Serial.print("Brake Mode: ");
        Serial.println(brakemotor_ ? "Enabled" : "Float");
    }
};

#endif
