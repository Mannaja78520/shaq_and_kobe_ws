#!/usr/bin/env python3

from pynput import keyboard
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray 
from geometry_msgs.msg import Twist
from rclpy import qos
from src.utilize import * 
from src.controller import *
import time 

class servo_position(Node):

    def __init__(self):
         super().__init__("servo_position")


         self.servo1 : int = 0

         self.servo_active = False

         self.send_servo_speed = self.create_publisher(
            Int16MultiArray, "/kobe/cmd_slap/rpm", qos_profile=qos.qos_profile_system_default
         )
         self.create_subscription(
            Int16MultiArray, '/kobe/cmd_slap', self.cmd_slap, qos_profile=qos.qos_profile_system_default
         )

         
         self.sent_data_timer = self.create_timer(0.01, self.sendData)
        

    def cmd_slap(self, msg):
        cross = bool(msg.data[0])
        if cross:
            self.servo1 = 1500
        else:
            self.servo1 = 500
    def sendData(self):
        servo_position = Int16MultiArray()

        servo_position.data[0] = Int16MultiArray(self.servo1)

        self.send_shoot_speed.publish(servo_position)

def main():
    rclpy.init()

    sub = servo_position()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
