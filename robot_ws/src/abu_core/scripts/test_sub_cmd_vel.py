#!/usr/bin/env python3

from pynput import keyboard
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy import qos
from src.utilize import * 
from src.controller import * 

class test_sub_cmd_vel(Node):
    # PID controllers for motors
    controller1 = Controller(kp=1.0, ki=0.1, kd=0.05, errorTolerance=0.01)
    controller2 = Controller(kp=1.0, ki=0.1, kd=0.05, errorTolerance=0.01)
    controller3 = Controller(kp=1.0, ki=0.1, kd=0.05, errorTolerance=0.01)
    controller4 = Controller(kp=1.0, ki=0.1, kd=0.05, errorTolerance=0.01)

    moveSpeed: float = 0.0
    slideSpeed: float = 0.0
    turnSpeed: float = 0.0

    maxSpeed : int = 1023.0 # pwm
    max_linear_speed = 2.0  # m/s max
    motor1Speed : float = 0
    motor2Speed : float = 0
    motor3Speed : float = 0
    motor4Speed : float = 0

    wheel_radians = 0.05

    motor_speeds = [0.0, 0.0, 0.0, 0.0]
    encoder_speeds = [0.0, 0.0, 0.0, 0.0]  # Speeds in m/s


    def __init__(self):
        super().__init__("Test_sub_cmd_vel_Node")

        self.send_robot_speed = self.create_publisher(
            Twist, "/motor_speed", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )

        self.subscription = self.create_subscription(
            Twist, '/debug/move_encoder', self.encoder_callback, qos_profile=qos.qos_profile_system_default
        )

        self.sent_data_timer = self.create_timer(0.01, self.sendData)

    def encoder_callback(self, msg):
        self.encoder_speeds[0] = msg.linear.x * ((2 * 3.14 * self.wheel_radians) / 60)
        self.encoder_speeds[1] = msg.linear.y * ((2 * 3.14 * self.wheel_radians) / 60)
        self.encoder_speeds[2] = msg.linear.z * ((2 * 3.14 * self.wheel_radians) / 60)
        self.encoder_speeds[3] = msg.angular.x * ((2 * 3.14 * self.wheel_radians) / 60)
        
    def cmd_vel(self, msg):

        self.move_speed = clip(msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        self.slide_speed = clip(msg.linear.y, -self.max_linear_speed, self.max_linear_speed)
        self.turn_speed = clip(msg.angular.z, -self.max_linear_speed, self.max_linear_speed)
       

        # Calculate motor speeds directly in m/s
        self.motor_speeds[0] = self.move_speed + self.slide_speed - self.turn_speed
        self.motor_speeds[1] = self.move_speed + self.slide_speed + self.turn_speed
        self.motor_speeds[2] = self.move_speed - self.slide_speed - self.turn_speed
        self.motor_speeds[3] = self.move_speed - self.slide_speed + self.turn_speed

        # Normalize motor speeds to respect max linear speed
        max_motor_speed = max(abs(speed) for speed in self.motor_speeds)
        if max_motor_speed > self.max_linear_speed:
            self.motor_speeds = [speed / max_motor_speed * self.max_linear_speed for speed in self.motor_speeds]

        self.motor_speeds[0] = self.motor_speeds[0] / self.max_linear_speed * self.maxSpeed
        self.motor_speeds[1] = self.motor_speeds[1] / self.max_linear_speed * self.maxSpeed
        self.motor_speeds[2] = self.motor_speeds[2] / self.max_linear_speed * self.maxSpeed
        self.motor_speeds[3] = self.motor_speeds[3] / self.max_linear_speed * self.maxSpeed



    def sendData(self):
        motorspeed_msg = Twist()

        # Apply PID control to motor speeds
        motorspeed_msg.linear.x = float(self.controller1.Calculate(self.motor_speeds[0] - self.encoder_speeds[0]))
        motorspeed_msg.linear.y = float(self.controller2.Calculate(self.motor_speeds[1] - self.encoder_speeds[1]))
        motorspeed_msg.linear.z = float(self.controller3.Calculate(self.motor_speeds[2] - self.encoder_speeds[2]))
        motorspeed_msg.angular.x = float(self.controller4.Calculate(self.motor_speeds[3] - self.encoder_speeds[3]))

        self.send_robot_speed.publish(motorspeed_msg)

def main():
    rclpy.init()

    sub = test_sub_cmd_vel()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()