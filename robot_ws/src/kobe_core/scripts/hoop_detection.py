#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2 as cv
import os
import time
import threading
import torch
from ultralytics import YOLO
from cameracapture import CameraCapture  
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# Load YOLO model
username = os.getenv("USER")
model_path = f"/home/{username}/shaq_and_koby_ws/image_test/trainvschair.pt"
model = YOLO(model_path)

# Use CUDA if available
device = "cuda" if torch.cuda.is_available() else "cpu"


class mainRun(Node):
    def __init__(self):
        super().__init__("Hoop_Detection")

        self.bridge = CvBridge()

        self.x = 0.0
        self.y = 0.0
        self.center_x = 0.0
        self.center_y = 0.0

        # Publisher for detected hoop data
        self.sent_where_hoop = self.create_publisher(
            Twist, "/kobe/send_where_hoop", qos_profile=qos.qos_profile_sensor_data
        )

        self.subscription = self.create_subscription(
            Image,
            "/kobe/image_raw",  # Topic name from v4l2_camera
            self.image_callback,
            qos.qos_profile_sensor_data
        )
        
        # Timer for sending data
        self.sent_data_timer = self.create_timer(0.05, self.sendData)


    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        start_time = time.time()

        results = model.predict(
            frame,
            imgsz=300,
            conf=0.4,
            half=False,
            device=device,
            stream=True
        )

        for result in results:
            if len(result.boxes) > 0:
                self.x = float(result.boxes.xywh[0][0].item())
                self.y = float(result.boxes.xywh[0][1].item())
            else:
                self.x, self.y = 0.0, 0.0

        height, width = frame.shape[:2]
        self.center_x = float(width // 2)
        self.center_y = float(height // 2)

        process_time = time.time() - start_time
        # self.get_logger().info(f"Detection time: {process_time:.3f}s")


    def sendData(self):
        hoopdata_msg = Twist()
        hoopdata_msg.linear.x = float(self.x)
        hoopdata_msg.linear.y = float(self.y)
        hoopdata_msg.angular.x = float(127.0)
        hoopdata_msg.angular.y = float(self.center_y)

        self.sent_where_hoop.publish(hoopdata_msg)
        
def main():
    rclpy.init()
    sub = mainRun()
    rclpy.spin(sub)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
