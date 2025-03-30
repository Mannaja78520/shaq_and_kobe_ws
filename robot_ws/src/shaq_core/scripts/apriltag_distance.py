#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import apriltag

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        
        self.publisher_ = self.create_publisher(Float32, '/shaq/distance/kobe', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.TAG_SIZE = 0.087  # Tag size in meters
        self.FOCAL_LENGTH = 653  # Change this after calibration
        
        options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(options)
        
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz
    
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame")
            return
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        for detection in detections:
            corners = detection.corners.astype(int)
            perceived_width = np.linalg.norm(corners[0] - corners[1])
            perceived_height = np.linalg.norm(corners[1] - corners[2])
            perceived_size = (perceived_width + perceived_height) / 2
            distance = (self.TAG_SIZE * self.FOCAL_LENGTH) / perceived_size
            
            self.get_logger().info(f"AprilTag ID {detection.tag_id} Distance: {distance:.2f} m")
            
            # Publish distance
            msg = Float32()
            msg.data = distance
            self.publisher_.publish(msg)
        
    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
