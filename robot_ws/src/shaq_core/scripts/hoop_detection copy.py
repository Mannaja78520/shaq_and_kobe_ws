#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Twist
import os
import time
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import cv2
from ament_index_python.packages import get_package_share_directory


# Load YOLO ONNX model


# Get the share directory of the package
package_share_directory = get_package_share_directory("shaq_core")

# Construct the full path to the ONNX model
model_path = os.path.join(package_share_directory, "models", "trainvschair.onnx")

model = YOLO(model_path, task="detect")

# Set device (ONNX typically runs best on CPU or with TensorRT)
device = "cuda" if torch.cuda.is_available() else "cpu"
input_size = (640, 480)  # must match model training size


class mainRun(Node):
    def __init__(self):
        super().__init__("Hoop_Detection")

        self.bridge = CvBridge()

        self.x = 0.0
        self.y = 0.0
        self.center_x = 0.0
        self.center_y = 0.0
        self.led_state = False

        self.sent_where_hoop = self.create_publisher(
            Twist, "/shaq/send_where_hoop", qos_profile=qos.qos_profile_sensor_data
        )
        self.sent_led = self.create_publisher(
            Twist, "/shaq/led", qos_profile=qos.qos_profile_default
        )
        self.subscription = self.create_subscription(
            Image,
            "/shaq/image_raw",
            self.image_callback,
            qos.qos_profile_sensor_data
        )

        self.sent_data_timer = self.create_timer(0.05, self.sendData)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        # Resize image to match ONNX input size
        frame_resized = cv2.resize(frame, input_size)

        # Run detection using ONNX model
        results = model.predict(
            source=frame_resized,
            imgsz=input_size,
            device=device,
        )

        # Parse results
        for result in results:
            if len(result.boxes) > 0:
                self.x = float(result.boxes.xywh[0][0].item())
                self.y = float(result.boxes.xywh[0][1].item())
            else:
                self.x, self.y = 0.0, 0.0

        # Update center of image
        height, width = frame_resized.shape[:2]
        self.center_x = float(width // 2)
        self.center_y = float(height // 2)

    def sendData(self):
        hoopdata_msg = Twist()
        led_msg = Twist()

        hoopdata_msg.linear.x = self.x
        hoopdata_msg.linear.y = self.y
        hoopdata_msg.angular.x = self.center_x
        hoopdata_msg.angular.y = self.center_y

        self.led_state = 265 <= self.x <= 275
        led_msg.linear.x = float(self.led_state)

        self.sent_led.publish(led_msg)
        self.sent_where_hoop.publish(hoopdata_msg)


def main():
    rclpy.init()
    node = mainRun()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()



