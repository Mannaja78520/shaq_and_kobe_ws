#ifndef DRIVE_OUTPUT_H
#define DRIVE_OUTPUT_H


#define MOTOR_MAX_RPM 750                                               // motor's max RPM          
#define MAX_RPM_RATIO 0.85                                              // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO          
#define MOTOR_OPERATING_VOLTAGE 24                                      // motor's operating voltage (used to calculate max RPM)
#define MOTOR_POWER_MAX_VOLTAGE 24                                      // max voltage of the motor's power source (used to calculate max RPM)
#define MOTOR_POWER_MEASURED_VOLTAGE 24                                 // current voltage reading of the power connected to the motor (used for calibration)
#define ENCODER1_PULSES_PER_REVOLUTION 2500                             // encoder 1 pulse
#define ENCODER2_PULSES_PER_REVOLUTION 2500                             // encoder 2 pulse
#define ENCODER_TICKS 4                                                 // encoder ticks
#define COUNTS_PER_REV1 ENCODER1_PULSES_PER_REVOLUTION * ENCODER_TICKS  // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 ENCODER1_PULSES_PER_REVOLUTION * ENCODER_TICKS  // wheel2 encoder's no of ticks per rev


#define PWM_FREQUENCY 20000
#define PWM_BITS 10


#define MOTOR1_INV false
#define MOTOR2_INV false
#define MOTOR3_INV true


#define MOTOR1_BREAK false
#define MOTOR2_BREAK false
#define MOTOR3_BREAK true


#define MOTORSHOOTER1_PWM 0
#define MOTORSHOOTER1_IN_A 1
#define MOTORSHOOTER1_IN_B 2


#define MOTORSHOOTER2_PWM 3
#define MOTORSHOOTER2_IN_A 4 
#define MOTORSHOOTER2_IN_B 5


#define MOTORLIFT_PWM 6
#define MOTORLIFT_IN_A 7
#define MOTORLIFT_IN_B 8

// Encoder 1 Parameter Upper Motor
#define MOTOR1_ENCODER_PIN_A 21
#define MOTOR1_ENCODER_PIN_B 20

// Encoder 2 Parameter Lower Motor
#define MOTOR2_ENCODER_PIN_A 17
#define MOTOR2_ENCODER_PIN_B 16


#endif
