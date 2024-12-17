#!/usr/bin/env python3

from pynput import keyboard
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy import qos
from src.utilize import * 


class keyboard_control(Node):

    moveSpeed: float = 0.0
    slideSpeed: float = 0.0
    turnSpeed: float = 0.0

    plusMoveSpeed: float = 0.01
    plusSlideSpeed: float = 0.01
    plusturnSpeed: float = 0.01
    plusSpeedSize = 0.01
    maxSpeed : float = 2.0 # m/s

    def __init__(self):
        super().__init__("KeyboardControl_Node")

        self.send_robot_speed = self.create_publisher(
            Twist, "/cmd_vel", qos_profile=qos.qos_profile_system_default
        )

        self.send_keyboard = self.create_publisher(
            String, '/keyboard', 10
        )

        self.sent_data_timer = self.create_timer(0.01, self.sendData)

    def on_press(self, key):
        try:
            keyboard_msg = String()

            # Adjust movement, sliding, and turning speeds
            self.moveSpeed = self.moveSpeed + self.plusMoveSpeed if key.char == 'w' else self.moveSpeed - self.plusMoveSpeed if key.char == 's' else self.moveSpeed
            self.slideSpeed = self.slideSpeed + self.plusSlideSpeed if key.char == 'd' else self.slideSpeed - self.plusSlideSpeed if key.char == 'a' else self.slideSpeed
            self.turnSpeed = self.turnSpeed + self.plusturnSpeed if key.char == 'e' else self.turnSpeed - self.plusturnSpeed if key.char == 'q' else self.turnSpeed

            self.plusMoveSpeed = self.plusMoveSpeed + self.plusSpeedSize if key.char == 'y' else self.plusMoveSpeed - self.plusSpeedSize if key.char == 'h' else self.plusMoveSpeed
            self.plusSlideSpeed = self.plusSlideSpeed + self.plusSpeedSize if key.char == 'u' else self.plusSlideSpeed - self.plusSpeedSize if key.char == 'j' else self.plusSlideSpeed
            self.plusturnSpeed = self.plusturnSpeed + self.plusSpeedSize if key.char == 'i' else self.plusturnSpeed - self.plusSpeedSize if key.char == 'k' else self.plusturnSpeed

            if key.char == 'b':
                self.moveSpeed = 0.0
                self.slideSpeed = 0.0
                self.turnSpeed = 0.0

            # Clip values
            self.moveSpeed = clip(self.moveSpeed, -self.maxSpeed, self.maxSpeed)
            self.slideSpeed = clip(self.slideSpeed, -self.maxSpeed, self.maxSpeed)
            self.turnSpeed = clip(self.turnSpeed, -self.maxSpeed, self.maxSpeed)
            self.plusMoveSpeed = clip(self.plusMoveSpeed, 0, self.maxSpeed)
            self.plusSlideSpeed = clip(self.plusSlideSpeed, 0, self.maxSpeed)
            self.plusturnSpeed = clip(self.plusturnSpeed, 0, self.maxSpeed)

            keyboard_msg.data = str(key.char)
            self.send_keyboard.publish(keyboard_msg)

        except AttributeError:
            return

    def on_release(self, key):
        if key == keyboard.Key.esc:
            self.get_logger().info('Exiting...')
            rclpy.shutdown()

    def start_listening(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            self.get_logger().info('Listening for keyboard input...')
            listener.join()

    def sendData(self):
        cmd_vel_msg = Twist()

        log_message = (
            "Use the following keys to control the robot: \n"
            "'w' : Increase move speed, \n"
            "'s' : Decrease move speed, \n"
            "'a' : Slide left, \n"
            "'d' : Slide right, \n"
            "'q' : Turn left, \n"
            "'e' : Turn right, \n"
            "'y' : Increase move speed increment. \n"
            "'h' : Decrease move speed increment. \n"
            "'u' : Increase slide speed increment. \n"
            "'j' : Decrease slide speed increment. \n"
            "'i' : Increase turn speed increment. \n"
            "'k' : Decrease turn speed increment. \n"
            "'b' : Brake. \n"
            f"Current Speeds: Move={self.moveSpeed:.2f}, Slide={self.slideSpeed:.2f}, Turn={self.turnSpeed:.2f}\n"
            f"Current Speed increment: Move={self.plusMoveSpeed:.2f}, Slide={self.plusSlideSpeed:.2f}, Turn={self.plusturnSpeed:.2f}"
        )

        # Log the combined message
        self.get_logger().info(log_message)

        # Assign speeds to the Twist message, ensuring they're floats
        cmd_vel_msg.linear.x = float(self.slideSpeed)
        cmd_vel_msg.linear.y = float(self.moveSpeed)
        cmd_vel_msg.angular.z = float(self.turnSpeed)

        # Publish the message
        self.send_robot_speed.publish(cmd_vel_msg)


def main():
    rclpy.init()

    sub = keyboard_control()
    sub_thread = threading.Thread(target=sub.start_listening)
    sub_thread.start()

    rclpy.spin(sub)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
