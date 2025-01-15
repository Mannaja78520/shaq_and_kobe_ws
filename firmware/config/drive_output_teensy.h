#ifndef DRIVE_OUTPUT_TEENSY_H
#define DRIVE_OUTPUT_TEENSY_H

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

#define PWM_FREQUENCY 20000
#define PWM_BITS 10

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV true
#define MOTOR3_INV false
#define MOTOR4_INV true

#define MOTOR1_BREAK true
#define MOTOR2_BREAK true
#define MOTOR3_BREAK true
#define MOTOR4_BREAK true

#define MOTOR1_PWM 0
#define MOTOR1_IN_A 1
#define MOTOR1_IN_B 2

#define MOTOR2_PWM 3
#define MOTOR2_IN_A 4
#define MOTOR2_IN_B 5

#define MOTOR3_PWM 6
#define MOTOR3_IN_A 7 
#define MOTOR3_IN_B 8

#define MOTOR4_PWM 9
#define MOTOR4_IN_A 10
#define MOTOR4_IN_B 11

#endif