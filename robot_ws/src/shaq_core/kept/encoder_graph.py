#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Updated message type
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time

class TwistSubscriber(Node):
    def __init__(self):
        super().__init__('encoder_sub')
        self.subscription = self.create_subscription(
            Twist,  
            '/shaq/cmd_shoot/rpm',  # Update to appropriate topic
            self.listener_callback,
            10)
        self.subscription  
        
        self.times = []
        self.linear_x_values = []
        self.linear_y_values = []
        self.start_time = time.time()

    def listener_callback(self, msg):
        current_time = time.time() - self.start_time
        self.times.append(current_time)
        linear_x_value = max(-1023, min(1023, msg.linear.x))  # Clamping between 0 and 1023
        linear_y_value = max(0, min(1023, msg.linear.y))  # Clamping between 0 and 1023
        self.linear_x_values.append(linear_x_value)
        self.linear_y_values.append(linear_y_value)
        self.get_logger().info(f'Received: Linear X={linear_x_value}, Linear Y={linear_y_value} at {current_time:.2f} seconds')

    def get_data(self):
        return self.times, self.linear_x_values, self.linear_y_values

def animate(i, node, line1, line2, ax):
    times, linear_x_values, linear_y_values = node.get_data()
    if times:
        ax.set_xlim(max(0, times[-1] - 10), times[-1] + 1)
    
    line1.set_xdata(times)
    line1.set_ydata(linear_x_values)
    line2.set_xdata(times)
    line2.set_ydata(linear_y_values)
    return line1, line2

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = TwistSubscriber()
    
    fig, ax = plt.subplots()
    line1, = ax.plot([], [], 'r-', label='Linear X Velocity (0-1023)')
    line2, = ax.plot([], [], 'b-', label='Linear Y Velocity (0-1023)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity')
    ax.legend()
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 1023)

    ani = animation.FuncAnimation(fig, animate, fargs=(node, line1, line2, ax), interval=500)
    
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()
    
    plt.show()
    
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
