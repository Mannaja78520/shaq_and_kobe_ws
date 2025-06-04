#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import time
from threading import Lock, Thread

class Show_Camera_Node(Node):
    def __init__(self):
        super().__init__("ShowCamera")

        self.bridge = CvBridge()

        self.lock = Lock()
        self.latest_raw = None
        self.latest_annotated = None
        self.last_annotated_time = 0
        self.annotated_timeout = 2.0
        best_effort_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.annotated_sub = self.create_subscription(
            Image,
            "/shaq/image/annotated_image",
            self.annotated_image_callback,
            qos_profile=best_effort_qos
        )
        
        self.raw_sub = self.create_subscription(
            Image,
            "/shaq/image_raw",
            self.raw_image_callback,
            10
        )

        self.running = True
        self.display_thread = Thread(target=self.display_loop)
        self.display_thread.start()

    def annotated_image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.latest_annotated = frame
                self.last_annotated_time = time.time()
        except Exception as e:
            self.get_logger().error(f"Annotated CvBridge error: {e}")

    def raw_image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.latest_raw = frame
        except Exception as e:
            self.get_logger().error(f"Raw CvBridge error: {e}")

    def display_loop(self):
        while self.running and rclpy.ok():
            frame_to_show = None
            with self.lock:
                # Show annotated image if recent enough
                if (self.latest_annotated is not None and 
                    (time.time() - self.last_annotated_time) < self.annotated_timeout):
                    frame_to_show = self.latest_annotated
                elif self.latest_raw is not None:
                    frame_to_show = self.latest_raw

            if frame_to_show is not None:
                cv.imshow("Camera", frame_to_show)
                cv.waitKey(1)  # Required for imshow to update

            time.sleep(0.03)  # ~30 FPS display rate

    def destroy_node(self):
        self.running = False
        self.display_thread.join()
        super().destroy_node()


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
