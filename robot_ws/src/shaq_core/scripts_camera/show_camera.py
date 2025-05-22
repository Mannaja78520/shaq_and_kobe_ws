#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import time
from threading import Lock

class Show_Camera_Node(Node):
    def __init__(self):
        super().__init__("ShowCamera")

        self.bridge = CvBridge()
        self.last_annotated_time = 0  # Time when last annotated image was received
        self.annotated_timeout = 2.0  # Seconds to wait before falling back to raw image
        self.processing = False
        self.lock = Lock()


        self.annotated_sub = self.create_subscription(
            Image,
            "/shaq/image/annotated_image",
            self.annotated_image_callback,
            qos.qos_profile_sensor_data
        )
        
        self.raw_sub = self.create_subscription(
            Image,
            "/shaq/image_raw",
            self.image_callback,
            qos.qos_profile_sensor_data
        )

    def image_callback(self, msg):
        with self.lock:
            if self.processing:
                return  # skip if busy
            self.processing = True
        start_time = time.time()

        # Only show raw image if annotated image hasn't been shown recently
        if time.time() - self.last_annotated_time < self.annotated_timeout:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            cv.imshow("Camera", frame)
            cv.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Raw CvBridge error: {e}")
        
        self.processing = False

    def annotated_image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_annotated_time = time.time()
            cv.imshow("Camera", frame)
            cv.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Annotated CvBridge error: {e}")

def main():
    rclpy.init()
    node = Show_Camera_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()

if __name__ == "__main__":
    main()
