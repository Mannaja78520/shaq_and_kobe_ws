#ifndef ENCODER_INPUT_H
#define ENCODER_INPUT_H

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/ACKERMANN)
    MOTOR3  MOTOR4  (4WD/MECANUM)
         BACK
*/

// INVERT MOTOR DIRECTIONS
#define ENCODER1_INV true
#define ENCODER2_INV false
#define ENCODER3_INV true 
#define ENCODER4_INV false

#define ENCODER1_INCRIMENT 32
#define ENCODER1_PIN_A 33
#define ENCODER1_PIN_B 25

#define ENCODER2_INCRIMENT 26
#define ENCODER2_PIN_A 27
#define ENCODER2_PIN_B 14

#define ENCODER3_INCRIMENT 23
#define ENCODER3_PIN_A 22 
#define ENCODER3_PIN_B 21

#define ENCODER4_INCRIMENT 19
#define ENCODER4_PIN_A 5
#define ENCODER4_PIN_B 4

/*
ROBOT ORIENTATION
         FRONT
       SPIN_BALL
         BACK
*/

// #define spinBall_INV false
// #define spinBall_BREAK false

// #define spinBall_PWM 17
#endif