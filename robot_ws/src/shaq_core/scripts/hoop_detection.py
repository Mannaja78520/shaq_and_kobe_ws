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

# Load YOLO model
username = os.getenv("USER")
model_path = f"/home/{username}/shaq_and_koby_ws/image_test/trainvschair.pt"
model = YOLO(model_path)

# Use CUDA if available
device = "cuda" if torch.cuda.is_available() else "cpu"

# Initialize Camera
camcap = CameraCapture(camera_index=0, width=320, height=240)  # Lower resolution for speed

class mainRun(Node):
    def __init__(self):
        super().__init__("Hoop_Detection")

        self.x = 0.0
        self.y = 0.0
        self.center_x = 0.0
        self.center_y = 0.0

        # Publisher for detected hoop data
        self.sent_where_hoop = self.create_publisher(
            Twist, "/shaq/send_where_hoop", qos_profile=qos.qos_profile_sensor_data
        )
        
        # Timer for sending data
        self.sent_data_timer = self.create_timer(0.05, self.sendData)

        # Thread for object detection
        self.detection_thread = threading.Thread(target=self.detectHoopLoop, daemon=True)
        self.detection_thread.start()

    def detectHoopLoop(self):
        """ Runs YOLO detection in a separate thread to improve performance """
        while rclpy.ok():
            start_time = time.time()
            screenshot = camcap.get_screenshot()  # Get frame from camera

            # Run YOLO with optimizations
            results = model.predict(
                screenshot,
                imgsz=224,  # Reduce input size
                conf=0.4,  # Lower confidence threshold
                half=False,  # Use half precision (if supported)
                device=device,
                stream=True  # Stream mode for faster processing
            )

            for result in results:  # ✅ Iterate over the generator
                if len(result.boxes) > 0:
                    self.x = float(result.boxes.xywh[0][0].item())
                    self.y = float(result.boxes.xywh[0][1].item())
                else:
                    self.x, self.y = 0.0, 0.0

            # Compute center of image
            height, width = screenshot.shape[:2]
            self.center_x = float(width // 2)
            self.center_y = float(height // 2)

            process_time = time.time() - start_time
            self.get_logger().info(f"Detection time: {process_time:.3f}s")

    def sendData(self):
        hoopdata_msg = Twist()
        hoopdata_msg.linear.x = float(self.x)
        hoopdata_msg.linear.y = float(self.y)
        hoopdata_msg.angular.x = float(139.0)
        hoopdata_msg.angular.y = float(self.center_y)

        self.sent_where_hoop.publish(hoopdata_msg)
        
def main():
    rclpy.init()
    sub = mainRun()
    rclpy.spin(sub)
    camcap.release()  # Close camera
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
