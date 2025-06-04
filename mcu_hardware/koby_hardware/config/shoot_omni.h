#ifndef DRIVE_OUTPUT_H
#define DRIVE_OUTPUT_H


#define PWM_FREQUENCY 20000
#define PWM_BITS 10
#define PWM_Max pow(2, PWM_BITS) - 1
#define PWM_Min -PWM_Max


#define MOTOR1_INV true
#define MOTOR2_INV false
#define MOTOR3_INV true


#define MOTOR1_BREAK false
#define MOTOR2_BREAK false
#define MOTOR3_BREAK true

//Lower
#define MOTORSHOOTER1_PWM 37     
#define MOTORSHOOTER1_IN_A 36
#define MOTORSHOOTER1_IN_B 35

//Upper
#define MOTORSHOOTER2_PWM 15
#define MOTORSHOOTER2_IN_A 14 
#define MOTORSHOOTER2_IN_B 13

//Feed
#define MOTORLIFT_PWM 12
#define MOTORLIFT_IN_A 11
#define MOTORLIFT_IN_B 10



#endif
