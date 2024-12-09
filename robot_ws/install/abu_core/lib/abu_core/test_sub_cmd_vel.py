#!/usr/bin/env python3

from pynput import keyboard
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy import qos


class test_sub_cmd_vel(Node):

    moveSpeed: float = 0.0
    slideSpeed: float = 0.0
    turnSpeed: float = 0.0

    maxSpeed : int = 1024 # pwm

    def __init__(self):
        super().__init__("Test_sub_cmd_vel_Node")

        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel, 10
        )
        
    def cmd_vel(self, msg):
        self.moveSpeed = msg.linear.x
        self.slideSpeed = msg.linear.y
        self.turnSpeed = msg.angular.z
        
        D = max(abs(self.moveSpeed)+abs(self.slideSpeed)+abs(self.turnSpeed), 2.0)
        motor1Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed + self.turnSpeed) / D * self.maxSpeed))
        motor2Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed - self.turnSpeed) / D * self.maxSpeed))
        motor3Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed + self.turnSpeed) / D * self.maxSpeed))
        motor4Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed - self.turnSpeed) / D * self.maxSpeed))
        print("motor 1 : " + str(motor1Speed),"motor 2 : " +  str(motor2Speed),"motor 3 : " +  str(motor3Speed),"motor 4 : " +  str(motor4Speed))

def main():
    rclpy.init()

    sub = test_sub_cmd_vel()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
