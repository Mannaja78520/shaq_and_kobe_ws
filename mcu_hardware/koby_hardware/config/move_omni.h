#ifndef DRIVE_OUTPUT_H
#define DRIVE_OUTPUT_H

#define PWM_FREQUENCY 20000
#define PWM_BITS 10
#define PWM_Max pow(2, PWM_BITS) - 1
#define PWM_Min -PWM_Max


//------------------------MOVE MOTOR-------------------------

//  Motor Brake
#define MOTORMOVE1_BRAKE true
#define MOTORMOVE2_BRAKE true
#define MOTORMOVE3_BRAKE true
#define MOTORMOVE4_BRAKE true

// INVERT MOTOR DIRECTIONS
#define MOTORMOVE1_INV true
#define MOTORMOVE2_INV false
#define MOTORMOVE3_INV true
#define MOTORMOVE4_INV false

/*

        (Motor2)//-------------\\(Motor1)
                |               |
                |               |
                |               |
                |               |               
        (Motor3)\\-------------//(Motor4)

*/

// Motor 1 Parameters
#define MOTOR1_PWM  0
#define MOTOR1_IN_A 1
#define MOTOR1_IN_B 2

// Motor 2 Parameters
#define MOTOR2_PWM  3
#define MOTOR2_IN_A 4
#define MOTOR2_IN_B 5

// Motor 3 Parameters
#define MOTOR3_PWM  36
#define MOTOR3_IN_A 37
#define MOTOR3_IN_B 38

// Motor 4 Parameters
#define MOTOR4_PWM  33
#define MOTOR4_IN_A 34
#define MOTOR4_IN_B 35


// I2C communication
#define SCL_PIN 19
#define SDA_PIN 18

#endif
