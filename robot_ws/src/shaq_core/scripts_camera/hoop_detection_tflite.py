#!/usr/bin/env python3

import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import numpy as np
import cv2
import time
from ament_index_python.packages import get_package_share_directory
import tensorflow.lite as tflite  # or use `import tensorflow.lite as tflite` if using full TensorFlow

# Load TFLite model
package_share_directory = get_package_share_directory("shaq_core")
model_path = os.path.join(package_share_directory, "models", "trainvschair_float16.tflite")

interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_size = (256, 256)


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
        self.image_pub = self.create_publisher(
            Image, "/shaq/image/annotated_image", qos_profile=qos.qos_profile_sensor_data
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
        start_time = time.time()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        frame_resized = cv2.resize(frame, input_size)
        h_resized, w_resized = frame_resized.shape[:2]

        # Prepare input for TFLite model
        input_tensor = frame_resized.astype(np.float32) / 255.0
        input_tensor = np.expand_dims(input_tensor, axis=0)  # NHWC
        if input_details[0]['dtype'] == np.uint8:
            input_tensor = (input_tensor * 255).astype(np.uint8)

        interpreter.set_tensor(input_details[0]['index'], input_tensor)
        interpreter.invoke()

        output = interpreter.get_tensor(output_details[0]['index'])  # shape: (N, 5)
        output = np.squeeze(output, axis=0).T  # shape (5, N)

        detections = [det for det in output if det[4] > 0.35]

        if detections:
            best_det = max(detections, key=lambda d: d[4])
            x, y, w, h, conf = best_det
            # print(conf)

            x1 = int((x - w / 2) * w_resized)
            y1 = int((y - h / 2) * h_resized)
            x2 = int((x + w / 2) * w_resized)
            y2 = int((y + h / 2) * h_resized)

            self.x = x * w_resized
            self.y = y * h_resized

            cv2.rectangle(frame_resized, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame_resized, f"hoop: {conf:.2f}", (x1, max(y1 - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        else:
            self.x, self.y = 0.0, 0.0

        self.center_x = w_resized / 2.0
        self.center_y = h_resized / 2.0

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame_resized, encoding="bgr8")
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Image publishing error: {e}")
        
        elapsed_ms = (time.time() - start_time) * 1000.0
        # print((time.time() - start_time) * 1000.0)
        self.get_logger().info(f"Inference time: {elapsed_ms:.2f} ms")

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
