#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time as ROS2Time
from std_msgs.msg import Header, Int16MultiArray
from sensor_msgs.msg import TimeReference
from rclpy import qos
from builtin_interfaces.msg import Time

class cmd_kobe_task(Node):

    def __init__(self):
        super().__init__("cmd_kobe_task")

        self.shaq_cmd_kobe_jump = False 

        self.send_kobe_task = self.create_publisher(
            TimeReference, "shaq/cmd_kobe/task", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Int16MultiArray, 'shaq/cmd_kobe', self.cmd_kobe, qos_profile=qos.qos_profile_system_default
        )

        self.sent_data_timer = self.create_timer(0.01, self.sendData)

    def cmd_kobe(self, msg):
        self.shaq_cmd_kobe_jump = msg.data[0]

    def sendData(self):
        if self.shaq_cmd_kobe_jump:
            kobe_task_msg = TimeReference()
            kobe_task_msg.header = Header()
            kobe_task_msg.header.stamp = self.get_clock().now().to_msg()
            kobe_task_msg.header.frame_id = "kobe_know"
            
            kobe_task_msg.time_ref = self.get_clock().now().to_msg()
            
            kobe_task_msg.source = "jump"
            
            self.send_kobe_task.publish(kobe_task_msg)

def main():
    rclpy.init()
    try:
        sub = cmd_kobe_task()
        rclpy.spin(sub)
    except KeyboardInterrupt:
        pass
    finally:
        sub.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
