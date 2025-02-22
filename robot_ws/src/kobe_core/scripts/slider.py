#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time as ROS2Time
from std_msgs.msg import Header, Int16MultiArray, Float32MultiArray
from sensor_msgs.msg import TimeReference
from rclpy import qos
from builtin_interfaces.msg import Time
import matplotlib.pyplot as plt
import numpy as np

from matplotlib.widgets import Button, Slider ,TextBox

class cmd_rotate(Node):

    def __init__(self):
        super().__init__("cmd_rotate")

        self.get_robot_angle = False

        self.send_pid = self.create_publisher(
            Float32MultiArray, "/kobe/pid/rotate", qos_profile=qos.qos_profile_system_default
        )

        self.sent_data_timer = self.create_timer(0.01, self.sendData)

        # Initialize slider values
        # self.init_amplitude = 5
        self.init_k = 3
        self.init_error = 1
        self.init_base = 1
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
            valmin=0.1,
            valmax=30,
            valinit=self.init_k,
        )

        self.axkp = self.fig.add_axes([0.25, 0.3, 0.65, 0.03])
        self.kp_slider = Slider(
            ax=self.axkp,
            label='kp',
            valmin=0.1,
            valmax=30,
            valinit=self.init_k,
        )

        self.axkd = self.fig.add_axes([0.25, 0.2, 0.65, 0.03])
        self.kd_slider = Slider(
            ax=self.axkd,
            label='kd',
            valmin=0.1,
            valmax=30,
            valinit=self.init_k,
        )

        self.axki = self.fig.add_axes([0.25, 0.4, 0.65, 0.03])
        self.ki_slider = Slider(
            ax=self.axki,
            label='ki',
            valmin=0.1,
            valmax=30,
            valinit=self.init_k,
        )

        self.axerrorTolerance = self.fig.add_axes([0.25, 0.5, 0.65, 0.03])
        self.errorTolerance_slider = Slider(
            ax=self.axerrorTolerance,
            label='error tolerance',
            valmin=0.1,
            valmax=30,
            valinit=self.init_error
        )

        self.axbaseSpeed = self.fig.add_axes([0.25, 0.6, 0.65, 0.03])
        self.baseSpeed_slider = Slider(
            ax=self.axbaseSpeed,
            label='baseSpeed',
            valmin=0.1,
            valmax=30,
            valinit=self.init_base,
        )

        self.axsetpoint = self.fig.add_axes([0.25, 0.7, 0.65, 0.03])
        self.setpoint_slider = Slider(
            ax=self.axsetpoint,
            label='setpoint',
            valmin=0.1,
            valmax=30,
            valinit=self.init_sp,
        )

        # # Make a vertically oriented slider to control the amplitude
        # self.axamp = self.fig.add_axes([0.1, 0.25, 0.0225, 0.63])
        # self.amp_slider = Slider(
        #     ax=self.axamp,
        #     label="Amplitude",
        #     valmin=0,
        #     valmax=10,
        #     valinit=self.init_amplitude,
        #     orientation="vertical"
        # )

        # Create a text box to show frequency and amplitude values in the x-axis region
        self.kf_text = self.ax.text(10, 10, f'kf: {self.init_k} ', 
                                      horizontalalignment='center', verticalalignment='center')
        self.kd_text = self.ax.text(10, 9, f'kd: {self.init_k}', 
                                      horizontalalignment='center', verticalalignment='center')
        self.ki_text= self.ax.text(18, 9, f'ki: {self.init_k}', 
                                      horizontalalignment='center', verticalalignment='center')
        self.kp_text = self.ax.text(0.5, 8, f'kp: {self.init_k}', 
                                      horizontalalignment='center', verticalalignment='center')
        self.errorTolerance_text = self.ax.text(0.5, 9, f'errorTolerance: {self.init_error}', 
                                      horizontalalignment='center', verticalalignment='center')
        self.baseSpeed_text = self.ax.text(0.5, 10, f'baseSpeed: {self.init_base}', 
                                      horizontalalignment='center', verticalalignment='center')
        self.sp_text = self.ax.text(10, 8, f'setpoint: {self.init_sp}', 
                                      horizontalalignment='center', verticalalignment='center')


        # self.amp_text = self.ax.text(0.5, 7, f'Amplitude: {self.init_amplitude}', 
        #                              horizontalalignment='center', verticalalignment='center')

        # Textboxes to show the current value of each slider
        # resetax = self.fig.add_axes([0.4, 0.025, 0.1, 0.04])
        self.axsetpoint_text = self.fig.add_axes([0.7, 0.85, 0.2, 0.05])  # Top right corner
        self.setpoint_textbox = TextBox(self.axsetpoint_text, 'Setpoint: ')
        self.setpoint_textbox.on_submit(self.update_setpoint)

        # Register the update function with each slider
        self.kf_slider.on_changed(self.update)
        # self.amp_slider.on_changed(self.update)
        self.kp_slider.on_changed(self.update)
        self.kd_slider.on_changed(self.update)
        self.ki_slider.on_changed(self.update)
        self.errorTolerance_slider.on_changed(self.update)
        self.baseSpeed_slider.on_changed(self.update)
        self.setpoint_slider.on_changed(self.update)


        # Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
        resetax = self.fig.add_axes([0.8, 0.025, 0.1, 0.04])
        self.button = Button(resetax, 'Reset', hovercolor='0.975')
        self.button.on_clicked(self.reset)

        # Show the plot
        plt.show()

    def update(self, val):
        # Update the text with the current frequency and amplitude values
        self.kd_text.set_text(f'kd: {self.kd_slider.val:.2f}')
        self.ki_text.set_text(f'ki: {self.ki_slider.val:.2f}')
        self.kp_text.set_text(f'kp: {self.kp_slider.val:.2f}')
        self.kf_text.set_text(f'kf: {self.kf_slider.val:.2f} ')
        self.errorTolerance_text.set_text(f'errorTolerance: {self.errorTolerance_slider.val:.2f} ')
        self.baseSpeed_text.set_text(f'baseSpeed: {self.baseSpeed_slider.val:.2f} ')
        self.sp_text.set_text(f'setpoint: {self.setpoint_slider.val:.2f}')
        # self.amp_text.set_text(f'Amplitude: {self.amp_slider.val:.2f}')
        self.fig.canvas.draw_idle()

    def reset(self, event):
        self.kf_slider.reset()
        self.kp_slider.reset()
        self.kd_slider.reset()
        self.ki_slider.reset()
        self.setpoint_slider.reset()
        self.errorTolerance_slider.reset()
        self.baseSpeed_slider.reset()
        # self.amp_slider.reset()

    def update_setpoint(self, text):
        """Update the setpoint value after input from the user."""
        try:
            # Convert the text input to a float and set it as the new value for the setpoint slider
            value = float(text)
            self.setpoint_slider.set_val(value)
        except ValueError:
            # If the input is invalid, we don't change the value
            print(f"Invalid input: {text}. Please enter a valid number.")


    def sendData(self):
        if self.get_robot_angle:
            shaq_task_msg = Float32MultiArray()
            shaq_task_msg.data = [self.kp_slider.val, self.ki_slider.val, self.kd_slider.val, self.kf_slider.val,self.errorTolerance_slider.val,self.baseSpeed_slider.val]
            
            self.send_pid.publish(shaq_task_msg)
    


def main():
    rclpy.init()
    sub = None  # Initialize sub to None to ensure it can be checked
    try:
        sub = cmd_rotate()  # Instantiate sub only if no exceptions before this point
        rclpy.spin(sub)
    except KeyboardInterrupt:
        pass
    finally:
        if sub:
            sub.destroy_node()  # Only call destroy_node if sub is instantiated
        rclpy.shutdown()


if __name__ == "__main__":
    main()
