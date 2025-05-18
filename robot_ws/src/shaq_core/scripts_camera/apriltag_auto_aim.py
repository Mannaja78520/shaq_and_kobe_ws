#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import apriltag
from geometry_msgs.msg import Twist

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        
        self.publisher_ = self.create_publisher(Twist, '/shaq/distance/kobe', 10)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/shaq/image_raw',
            self.image_callback,
            10)

        
        self.TAG_SIZE = 0.1  # Tag size in meters
        self.FOCAL_LENGTH = 653  # Change this after calibration
        
        options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(options)
        
        # self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz
    


    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        frame_h, frame_w = gray.shape
        SCREEN_CENTERS = {
            0: (320, 0),
            1: (148, 0),
            2: (340, 0),
            3: (350, 0)
        }
        default_center = (frame_w // 2, frame_h // 2)

        if len(detections) == 0:
            # self.get_logger().info("No AprilTag detected, sending position (0, 0)")
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            self.publisher_.publish(msg)
        else:
            for detection in detections:
                tag_id = detection.tag_id
                corners = detection.corners.astype(int)
                center = tuple(map(int, detection.center))
                screen_center = SCREEN_CENTERS.get(tag_id, default_center)

                perceived_width = np.linalg.norm(corners[0] - corners[1])
                perceived_height = np.linalg.norm(corners[1] - corners[2])
                perceived_size = (perceived_width + perceived_height) / 2
                distance = (self.TAG_SIZE * self.FOCAL_LENGTH) / perceived_size

                # self.get_logger().info(
                #     f"Tag ID {detection.tag_id} | Distance: {distance:.2f} m | Tag Center: ({center[0]}, {center[1]}) | Screen Center: {screen_center}"
                # )

                msg = Twist()
                msg.linear.x = float(center[0])
                msg.linear.y = float(center[1])
                msg.linear.z = distance
                msg.angular.x = float(screen_center[0])
                msg.angular.y = float(55555555555555)
                msg.angular.z = float(tag_id)
                self.publisher_.publish(msg)



        
    def destroy_node(self):
        # self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
