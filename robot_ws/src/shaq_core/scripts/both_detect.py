#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import apriltag
import torch
from ultralytics import YOLO
import time
import threading
import os
import queue

# Load YOLO model
username = os.getenv("USER")
model_path = f"/home/{username}/shaq_and_koby_ws/image_test/trainvschair.pt"
model = YOLO(model_path)
device = "cuda" if torch.cuda.is_available() else "cpu"

class IntegratedDetectionNode(Node):
    def __init__(self):
        super().__init__('integrated_detection_node')

        # Publishers for AprilTag and Hoop Detection
        self.publisher_apriltag = self.create_publisher(Twist, '/shaq/distance/kobe', 10)
        self.publisher_hoop = self.create_publisher(Twist, '/shaq/send_where_hoop', 10)
        
        # Initialize OpenCV bridge and AprilTag detector
        self.bridge = CvBridge()
        self.TAG_SIZE = 0.1  # Tag size in meters
        self.FOCAL_LENGTH = 653
        options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(options)
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().warn("Unable to open camera.")
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Frame queue for thread-safe access
        self.frame_queue = queue.Queue(maxsize=1)
        
        # Toggles for displaying the image (removed imshow)
        self.show_image = False  # Set to True to show the image by default

        # Initialize variables for hoop detection
        self.x, self.y = 0.0, 0.0

        # Timer for processing frames
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz

        # Start the YOLO detection in a separate thread
        self.detection_thread = threading.Thread(target=self.detect_hoop_loop, daemon=True)
        self.detection_thread.start()

    def capture_frame(self):
        """Captures a frame from the camera and stores it in the queue."""
        ret, frame = self.cap.read()
        if ret:
            if self.frame_queue.full():
                self.frame_queue.get()  # Remove the oldest frame if the queue is full
            self.frame_queue.put(frame)

    def detect_hoop_loop(self):
        """Runs YOLO detection in a separate thread for hoop detection"""
        while rclpy.ok():
            if not self.frame_queue.empty():
                frame = self.frame_queue.get()
                start_time = time.time()

                # Perform YOLO detection
                results = model(frame)
                for result in results:
                    if len(result.boxes) > 0:
                        self.x = float(result.boxes.xywh[0][0].item())
                        self.y = float(result.boxes.xywh[0][1].item())
                    else:
                        self.x, self.y = 0.0, 0.0

                # Log time taken for hoop detection
                process_time = time.time() - start_time
                self.get_logger().info(f"Detection time: {process_time:.3f}s")

    def process_frame(self):
        self.capture_frame()  # Capture a frame in the main thread

        if not self.frame_queue.empty():
            frame = self.frame_queue.get()
            # AprilTag detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(gray)

            frame_h, frame_w = gray.shape

            SCREEN_CENTERS = {
                0: (320, 240),    # Tag ID 0 uses center (320, 240)
                1: (300, 240),    # Tag ID 1 uses center (300, 240)
                2: (340, 240),    # Tag ID 2 uses center (340, 240)
                3: (350 ,250)
            }

            default_center = (frame_w // 2, frame_h // 2)

            # AprilTag detection logic (remaining same as your original code)
            for detection in detections:
                tag_id = detection.tag_id
                corners = detection.corners.astype(int)
                center = tuple(map(int, detection.center))

                screen_center = SCREEN_CENTERS.get(tag_id, default_center)

                # Calculate perceived size and distance
                perceived_width = np.linalg.norm(corners[0] - corners[1])
                perceived_height = np.linalg.norm(corners[1] - corners[2])
                perceived_size = (perceived_width + perceived_height) / 2
                distance = (self.TAG_SIZE * self.FOCAL_LENGTH) / perceived_size

                # Publish AprilTag data
                msg_apriltag = Twist()
                msg_apriltag.linear.x = float(center[0])
                msg_apriltag.linear.y = float(center[1])
                msg_apriltag.linear.z = distance

                msg_apriltag.angular.x = float(screen_center[0])  # Screen center x
                msg_apriltag.angular.y = float(screen_center[1])  # Screen center y
                msg_apriltag.angular.z = float(tag_id)
                self.publisher_apriltag.publish(msg_apriltag)

            # Publish Hoop data
            msg_hoop = Twist()
            msg_hoop.linear.x = self.x
            msg_hoop.linear.y = self.y
            msg_hoop.angular.x = 139.0
            msg_hoop.angular.y = float(frame_h // 2)  # Assuming the center y of the image
            self.publisher_hoop.publish(msg_hoop)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
