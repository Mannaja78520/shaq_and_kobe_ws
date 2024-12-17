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

    maxSpeed : int = 1023.0 # pwm
    motor1Speed : float = 0
    motor2Speed : float = 0
    motor3Speed : float = 0
    motor4Speed : float = 0

    def __init__(self):
        super().__init__("Test_sub_cmd_vel_Node")

        self.send_robot_speed = self.create_publisher(
            Twist, "/motor_speed", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel, 10
        )

        self.sent_data_timer = self.create_timer(0.01, self.sendData)
        
    def cmd_vel(self, msg):
        self.slideSpeed = msg.linear.x
        self.moveSpeed = msg.linear.y
        self.turnSpeed = msg.angular.z
        
        D = max(abs(self.moveSpeed)+abs(self.slideSpeed)+abs(self.turnSpeed), 2.0)
        self.motor1Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed + self.turnSpeed) / D * self.maxSpeed))
        self.motor2Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed - self.turnSpeed) / D * self.maxSpeed))
        self.motor3Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed + self.turnSpeed) / D * self.maxSpeed))
        self.motor4Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed - self.turnSpeed) / D * self.maxSpeed))
        print("motor 1 : " + str(self.motor1Speed),"motor 2 : " +  str(self.motor2Speed),"motor 3 : " +  str(self.motor3Speed),"motor 4 : " +  str(self.motor4Speed))


    def sendData(self):
        motorspeed_msg = Twist()
        motorspeed_msg.linear.x = float(-self.motor1Speed)
        motorspeed_msg.linear.y = float(-self.motor2Speed)
        motorspeed_msg.linear.z = float(-self.motor3Speed)
        motorspeed_msg.angular.x = float(-self.motor4Speed)
        self.send_robot_speed.publish(motorspeed_msg)

def main():
    rclpy.init()

    sub = test_sub_cmd_vel()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
