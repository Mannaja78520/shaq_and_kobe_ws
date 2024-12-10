#!/usr/bin/env python3
import rclpy
import math
import time

from rclpy import qos
from rclpy.node import Node
import rclpy.parameter
from std_msgs.msg import (
    String,
)
from geometry_msgs.msg import Twist
import numpy as np

class HelloWorld(Node):
    def __init__(self):
        super().__init__("HelloWorld_node")
        
        self.hello = self.create_publisher(
            String,
            "/helloWorld",
            qos_profile=qos.qos_profile_sensor_data,
        )
        
        self.sent_timer = self.create_timer(1, self.timer_callback)
        self.sent_a = self.create_timer(0.1, self.timer_a)
        # self.sent_timer = self.create_timer(0.1, self.timer_callback)
        self.i = 0
        
        
    def timer_callback(self):
        msg_HelloWorld = String()
        manpub = "Hello am earn " + str(self.i)
        msg_HelloWorld.data = manpub
        self.hello.publish(msg_HelloWorld)
        self.i = self.i + 1
        
    def timer_a(self):
        print("A")
        # self.send_twist_msg()
        
        
            
def main():
    rclpy.init()
    sub = HelloWorld()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
