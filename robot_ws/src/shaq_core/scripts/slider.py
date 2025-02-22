#!/usr/bin/env python3

import time
import rclpy
import threading
from rclpy.node import Node
from rclpy.time import Time as ROS2Time
from std_msgs.msg import Header, Int16MultiArray, Float32MultiArray
from sensor_msgs.msg import TimeReference
from rclpy import qos
from builtin_interfaces.msg import Time
import matplotlib.pyplot as plt
import numpy as np

from matplotlib.widgets import Button, Slider, TextBox

class cmd_rotate(Node):

    def __init__(self):
        super().__init__("cmd_rotate")

        self.send_pid = self.create_publisher(
            Float32MultiArray, "/shaq/pid/rotate", qos_profile=qos.qos_profile_system_default
        )

        self.sent_data_timer = self.create_timer(0.05, self.sendData)

        # Initialize slider values
        self.init_k = 0.0001
        self.init_error = 0.5
        self.init_base = 0
        self.init_sp = 0

        # Create the figure and axes without any plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 30)  # Set limits for the x-axis
        self.ax.set_ylim(0, 10)  # Set limits for the y-axis (but will not display any data)

        # Turn off the axes grid, ticks, labels, and the frame (border)
        self.ax.set_facecolor('white')  # Optional: ensure the background is white
        self.ax.grid(False)
        self.ax.set_xticks([])  # Remove x-ticks
        self.ax.set_yticks([])  # Remove y-ticks
        self.ax.spines['top'].set_visible(False)  # Remove the top border
        self.ax.spines['right'].set_visible(False)  # Remove the right border
        self.ax.spines['left'].set_visible(False)  # Remove the left border
        self.ax.spines['bottom'].set_visible(False)  # Remove the bottom border

        # Adjust the main plot to make room for the sliders
        self.fig.subplots_adjust(left=0.25, bottom=0.25)

        # Make a horizontal slider to control the frequency.
        self.axkf = self.fig.add_axes([0.25, 0.1, 0.65, 0.03])
        self.kf_slider = Slider(
            ax=self.axkf,
            label='kf',
            valmin=0.0001,
            valmax=30,
            valinit=self.init_k,
        )

        self.axkp = self.fig.add_axes([0.25, 0.3, 0.65, 0.03])
        self.kp_slider = Slider(
            ax=self.axkp,
            label='kp',
            valmin=0.0001,
            valmax=30,
            valinit=self.init_k,
        )

        self.axkd = self.fig.add_axes([0.25, 0.2, 0.65, 0.03])
        self.kd_slider = Slider(
            ax=self.axkd,
            label='kd',
            valmin=0.0001,
            valmax=30,
            valinit=self.init_k,
        )

        self.axki = self.fig.add_axes([0.25, 0.4, 0.65, 0.03])
        self.ki_slider = Slider(
            ax=self.axki,
            label='ki',
            valmin=0.0001,
            valmax=30,
            valinit=self.init_k,
        )

        self.axerrorTolerance = self.fig.add_axes([0.25, 0.5, 0.65, 0.03])
        self.errorTolerance_slider = Slider(
            ax=self.axerrorTolerance,
            label='error tolerance',
            valmin=0.0001,
            valmax=30,
            valinit=self.init_error
        )

        self.axbaseSpeed = self.fig.add_axes([0.25, 0.6, 0.65, 0.03])
        self.baseSpeed_slider = Slider(
            ax=self.axbaseSpeed,
            label='baseSpeed',
            valmin=0.0001,
            valmax=30,
            valinit=self.init_base,
        )

        self.axsetpoint = self.fig.add_axes([0.25, 0.7, 0.65, 0.03])
        self.setpoint_slider = Slider(
            ax=self.axsetpoint,
            label='setpoint',
            valmin=-1000,
            valmax=1000,
            valinit=self.init_sp,
        )

        # Create a text box to show frequency and amplitude values in the x-axis region
        self.kf_text = self.ax.text(-5, -2, f'kf: {self.init_k} ', 
                                      horizontalalignment='center', verticalalignment='center')
        self.kd_text = self.ax.text(-5, -1, f'kd: {self.init_k}', 
                                      horizontalalignment='center', verticalalignment='center')
        self.ki_text= self.ax.text(-5, 0, f'ki: {self.init_k}', 
                                      horizontalalignment='center', verticalalignment='center')
        self.kp_text = self.ax.text(-5, 1, f'kp: {self.init_k}', 
                                      horizontalalignment='center', verticalalignment='center')
        self.errorTolerance_text = self.ax.text(-5, 2, f'errorTolerance: {self.init_error}', 
                                      horizontalalignment='center', verticalalignment='center')
        self.baseSpeed_text = self.ax.text(-5, 3.15, f'baseSpeed: {self.init_base}', 
                                      horizontalalignment='center', verticalalignment='center')
        self.sp_text = self.ax.text(-5, -3, f'setpoint: {self.init_sp}', 
                                      horizontalalignment='center', verticalalignment='center')

        # Textboxes to show the current value of each slider
        self.kp_textbox = TextBox(self.fig.add_axes([0.3, 0.75, 0.1, 0.05]), 'kp: ')
        self.kp_textbox.on_submit(self.update_kp)

        self.kd_textbox = TextBox(self.fig.add_axes([0.1, 0.75, 0.1, 0.05]), 'kd: ')
        self.kd_textbox.on_submit(self.update_kd)

        self.errorTolerance_textbox = TextBox(self.fig.add_axes([0.5, 0.75, 0.1, 0.05]), 'error: ')
        self.errorTolerance_textbox.on_submit(self.update_errorTolerance)

        self.baseSpeed_textbox = TextBox(self.fig.add_axes([0.8, 0.75, 0.1, 0.05]), 'basespeed: ')
        self.baseSpeed_textbox.on_submit(self.update_baseSpeed)

        self.ki_textbox = TextBox(self.fig.add_axes([0.3, 0.9, 0.1, 0.05]), 'ki: ')
        self.ki_textbox.on_submit(self.update_ki)

        self.kf_textbox = TextBox(self.fig.add_axes([0.5, 0.9, 0.1, 0.05]), 'kf: ')
        self.kf_textbox.on_submit(self.update_kf)

        self.setpoint_textbox = TextBox(self.fig.add_axes([0.8, 0.9, 0.1, 0.05]), 'setpoint: ')
        self.setpoint_textbox.on_submit(self.update_setpoint)
        # self.kf_textbox.on_submit(self.update_kf)
        
        # Register the update function with each slider
        self.kf_slider.on_changed(self.update)
        self.kp_slider.on_changed(self.update)
        self.kd_slider.on_changed(self.update)
        self.ki_slider.on_changed(self.update)
        self.errorTolerance_slider.on_changed(self.update)
        self.baseSpeed_slider.on_changed(self.update)
        self.setpoint_slider.on_changed(self.update)

        # Create a reset button
        resetax = self.fig.add_axes([0.8, 0.025, 0.1, 0.04])
        self.button = Button(resetax, 'Reset', hovercolor='0.975')
        self.button.on_clicked(self.reset)

    def start_ros2(self):
        rclpy.spin(self)

    def start_matplotlib(self):
        plt.show()

    def update(self, val):
        # Update the text with the current frequency and amplitude values
        self.kd_text.set_text(f'kd: {self.kd_slider.val:.4f}')
        self.ki_text.set_text(f'ki: {self.ki_slider.val:.4f}')
        self.kp_text.set_text(f'kp: {self.kp_slider.val:.4f}')
        self.kf_text.set_text(f'kf: {self.kf_slider.val:.4f}')
        self.errorTolerance_text.set_text(f'errorTolerance: {self.errorTolerance_slider.val:.4f}')
        self.baseSpeed_text.set_text(f'baseSpeed: {self.baseSpeed_slider.val:.4f}')
        self.sp_text.set_text(f'setpoint: {self.setpoint_slider.val:.4f}')
        self.fig.canvas.draw_idle()
        
    def update_setpoint(self,text):
        """Update the setpoint vaule after input from the user."""
        try:
            value = float(text)
            self.setpoint_slider.set_val(value)
        except ValueError:
            pass

    def update_kp(self,text):
        """Update the setpoint vaule after input from the user."""
        try:
            value = float(text)
            self.kp_slider.set_val(value)
        except ValueError:
            pass
    
    def update_ki(self,text):
        """Update the setpoint vaule after input from the user."""
        try:
            value = float(text)
            self.ki_slider.set_val(value)
        except ValueError:
            pass
    
    def update_kd(self,text):
        """Update the setpoint vaule after input from the user."""
        try:
            value = float(text)
            self.kd_slider.set_val(value)
        except ValueError:
            pass

    def update_kf(self,text):
        """Update the setpoint vaule after input from the user."""
        try:
            value = float(text)
            self.kf_slider.set_val(value)
        except ValueError:
            pass

    def update_errorTolerance(self,text):
        """Update the setpoint vaule after input from the user."""
        try:
            value = float(text)
            self.errorTolerance_slider.set_val(value)
        except ValueError:
            pass

    
    def update_baseSpeed(self,text):
        """Update the setpoint vaule after input from the user."""
        try:
            value = float(text)
            self.baseSpeed_slider.set_val(value)
        except ValueError:
            pass


    def reset(self, event):
        self.kf_slider.reset()
        self.kp_slider.reset()
        self.kd_slider.reset()
        self.ki_slider.reset()
        self.setpoint_slider.reset()
        self.errorTolerance_slider.reset()
        self.baseSpeed_slider.reset()

    def sendData(self):
        shaq_task_msg = Float32MultiArray()
        shaq_task_msg.data = [
            float(self.kp_slider.val),
            float(self.ki_slider.val),
            float(self.kd_slider.val),
            float(self.kf_slider.val),
            float(self.errorTolerance_slider.val),
            float(self.baseSpeed_slider.val)
        ]
        
        self.get_logger().info(f"Sending data: {shaq_task_msg.data}")
        self.send_pid.publish(shaq_task_msg)

def main():
    # Initialize ROS2 and create the node
    rclpy.init()
    sub = cmd_rotate()

    # Create a separate thread for ROS2 to run in the background
    ros2_thread = threading.Thread(target=sub.start_ros2)
    ros2_thread.daemon = True  # Ensure it exits when the main program exits
    ros2_thread.start()

    # Run Matplotlib on the main thread
    sub.start_matplotlib()

    # After the plot window is closed, ensure ROS2 cleanup
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
