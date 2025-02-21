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

from matplotlib.widgets import Button, Slider

def f(t, amplitude, frequency):
    return amplitude * np.sin(2 * np.pi * frequency * t)


class cmd_rotate(Node):

    def __init__(self):
        super().__init__("cmd_rotate")

        self.get_robot_angle = False

        self.send_koby_task = self.create_publisher(
            Float32MultiArray, "shaq/pid/rotate", qos_profile=qos.qos_profile_system_default
        )

        self.sent_data_timer = self.create_timer(0.01, self.sendData)

        # Initialize the plotting components inside the class
        self.t = np.linspace(0, 1, 1000)

        # Define initial parameters
        self.init_amplitude = 5
        self.init_frequency = 3

        # Create the figure and the line that we will manipulate
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot(self.t, f(self.t, self.init_amplitude, self.init_frequency), lw=2)
        self.ax.set_xlabel('Time [s]')

        # Adjust the main plot to make room for the sliders
        self.fig.subplots_adjust(left=0.25, bottom=0.25)

        # Make a horizontal slider to control the frequency.
        self.axfreq = self.fig.add_axes([0.25, 0.1, 0.65, 0.03])
        self.freq_slider = Slider(
            ax=self.axfreq,
            label='Frequency [Hz]',
            valmin=0.1,
            valmax=30,
            valinit=self.init_frequency,
        )

        # Make a vertically oriented slider to control the amplitude
        self.axamp = self.fig.add_axes([0.1, 0.25, 0.0225, 0.63])
        self.amp_slider = Slider(
            ax=self.axamp,
            label="Amplitude",
            valmin=0,
            valmax=10,
            valinit=self.init_amplitude,
            orientation="vertical"
        )

        # Register the update function with each slider
        self.freq_slider.on_changed(self.update)
        self.amp_slider.on_changed(self.update)

        # Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
        resetax = self.fig.add_axes([0.8, 0.025, 0.1, 0.04])
        self.button = Button(resetax, 'Reset', hovercolor='0.975')
        self.button.on_clicked(self.reset)

        # Show the plot
        plt.show()

    def update(self, val):
        self.line.set_ydata(f(self.t, self.amp_slider.val, self.freq_slider.val))
        self.fig.canvas.draw_idle()

    def reset(self, event):
        self.freq_slider.reset()
        self.amp_slider.reset()

    def sendData(self):
        if self.get_robot_angle:
            shaq_task_msg = TimeReference()
            shaq_task_msg.header = Header()
            shaq_task_msg.header.stamp = self.get_clock().now().to_msg()
            shaq_task_msg.header.frame_id = "shaq_know"
            
            shaq_task_msg.time_ref = self.get_clock().now().to_msg()
            
            shaq_task_msg.source = "rotate"
            
            self.send_koby_task.publish(shaq_task_msg)


def main():
    rclpy.init()
    try:
        sub = cmd_rotate()
        rclpy.spin(sub)
    except KeyboardInterrupt:
        pass
    finally:
        sub.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
