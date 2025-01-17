#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy import qos
from src.utilize import *
from src.controller import *

class TestShooterSubCmdVel(Node):
    def __init__(self):
        super().__init__("test_shooter_sub_cmd_vel_node")
        
        # Instance variables
        self.controller1 = Controller(kp=1.0, ki=0.1, kd=0.05, errorTolerance=0.01)
        self.controller2 = Controller(kp=1.0, ki=0.1, kd=0.05, errorTolerance=0.01)
        
        self.maxSpeed = 1023.0  # PWM max
        self.max_linear_speed = 2.0  # m/s max
        
        self.motor_speeds = [0.0, 0.0]
        self.encoder_speeds = [0.0, 0.0]  # Speeds in m/s
        
        # ROS 2 publishers and subscribers
        self.send_robot_speed = self.create_publisher(
            Twist, "/motor_shooter_speed", qos_profile=qos.qos_profile_system_default
        )
        
        self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, qos_profile=qos.qos_profile_system_default
        )
        
        self.create_subscription(
            Twist, "/debug/shooter_encoder", self.encoder_callback, qos_profile=qos.qos_profile_system_default
        )
        
        self.sent_data_timer = self.create_timer(0.01, self.sendData)
    
    def encoder_callback(self, msg):
        # Update encoder speeds
        self.encoder_speeds[0] = msg.linear.x
        self.encoder_speeds[1] = msg.linear.y
    
    def cmd_vel_callback(self, msg):
        # Update motor speeds from cmd_vel
        self.motor_speeds[0] = msg.argular.y
        self.motor_speeds[1] = msg.argular.y
        
        # Normalize and scale motor speeds
        max_motor_speed = max(abs(speed) for speed in self.motor_speeds)
        if max_motor_speed > self.max_linear_speed:
            self.motor_speeds = [speed / max_motor_speed * self.max_linear_speed for speed in self.motor_speeds]
        
        self.motor_speeds = [
            speed / self.max_linear_speed * self.maxSpeed for speed in self.motor_speeds
        ]
    
    def sendData(self):
        # Apply PID control and publish motor speeds
        motorspeed_msg = Twist()
        motorspeed_msg.linear.x = float(self.controller1.Calculate(self.motor_speeds[0] - self.encoder_speeds[0]))
        motorspeed_msg.linear.y = float(self.controller2.Calculate(self.motor_speeds[1] - self.encoder_speeds[1]))
        self.send_robot_speed.publish(motorspeed_msg)

def main():
    rclpy.init()
    node = TestShooterSubCmdVel()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
