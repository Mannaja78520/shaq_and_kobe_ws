#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from rclpy import qos

class Gamepad:
    def __init__(self):
        self.lx : float = 0.0
        self.ly : float = 0.0
        self.lt : float = 0.0
        self.rx : float = 0.0
        self.ry : float = 0.0
        self.rt : float = 0.0
        # self.dpadLeftRight : float = 0.0
        # self.dpadUpDown : float = 0.0


class joystick(Node):
    def __init__(self):
        super().__init__("joystick")

        self.pub_cmd = self.create_publisher(
            Twist, "/cmd_vel", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Joy, '/joy', self.joy, 10
        )

        self.gamepad = Gamepad()
        self.maxspeed : float = 1.0

        self.sent_data_timer = self.create_timer(0.01, self.sendData)

    def joy(self, msg):
        self.gamepad.lx = float(msg.axes[0] * -1)
        self.gamepad.ly = float(msg.axes[1])
        self.gamepad.rx = float(msg.axes[3] * -1)
        self.gamepad.ry = float(msg.axes[4])
        # self.gamepad.lt = float((msg.axes[2] + 1) / 2)
        # self.gamepad.rt = float((msg.axes[5] + 1) / 2)
        # print(self.gamepad.lx, self.gamepad.ly, self.gamepad.rx)
        print(self.gamepad.ry)

    def sendData(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(self.gamepad.lx * self.maxspeed)
        cmd_vel_msg.linear.y = float(self.gamepad.ly * self.maxspeed)
        cmd_vel_msg.angular.x = float(self.gamepad.rx * self.maxspeed)
        cmd_vel_msg.angular.y = float(self.gamepad.ry * self.maxspeed)
        self.pub_cmd.publish(cmd_vel_msg)

def main():
    rclpy.init()

    sub = joystick()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
