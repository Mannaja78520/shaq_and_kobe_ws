#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist  
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time

class TwistSubscriber(Node):
    def __init__(self):
        super().__init__('encoder_sub')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # ลองเปลี่ยนเป็น RELIABLE หากไม่ได้ผล
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Twist,
            '/shaq/debug/cmd_shoot/rpm',
            self.listener_callback,
            qos_profile
        )
        
        self.times = []
        self.linear_x_values = []
        self.linear_y_values = []
        self.angular_x_values = []
        self.angular_y_values = []
        self.start_time = time.time()

    def listener_callback(self, msg):
        current_time = time.time() - self.start_time
        self.times.append(current_time)

        linear_x_value = max(-1023, min(1023, msg.linear.x))
        linear_y_value = max(0, min(1023, msg.linear.y))
        angular_x_value = max(-1023, min(1023, msg.angular.x))
        angular_y_value = max(-1023, min(1023, msg.angular.y))

        self.linear_x_values.append(linear_x_value)
        self.linear_y_values.append(linear_y_value)
        self.angular_x_values.append(angular_x_value)
        self.angular_y_values.append(angular_y_value)

        self.get_logger().info(
            f'Received: Linear X={linear_x_value}, Linear Y={linear_y_value}, '
            f'Angular X={angular_x_value}, Angular Y={angular_y_value} at {current_time:.2f} seconds'
        )

    def get_data(self):
        return self.times, self.linear_x_values, self.linear_y_values, self.angular_x_values, self.angular_y_values

def animate(i, node, line1, line2, line3, line4, ax):
    times, linear_x_values, linear_y_values, angular_x_values, angular_y_values = node.get_data()
    
    if times:
        ax.set_xlim(max(0, times[-1] - 10), times[-1] + 1)
    
    line1.set_xdata(times)
    line1.set_ydata(linear_x_values)
    line2.set_xdata(times)
    line2.set_ydata(linear_y_values)
    line3.set_xdata(times)
    line3.set_ydata(angular_x_values)
    line4.set_xdata(times)
    line4.set_ydata(angular_y_values)

    return line1, line2, line3, line4

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = TwistSubscriber()
    
    fig, ax = plt.subplots()
    line1, = ax.plot([], [], 'r-', label='Target_Upper')
    line2, = ax.plot([], [], 'b-', label='Target_Lower')
    line3, = ax.plot([], [], 'g-', label='Current_Upper')
    line4, = ax.plot([], [], 'm-', label='Current_Lower')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity')
    ax.legend()
    ax.set_xlim(0, 10)
    ax.set_ylim(-1023, 1023)  

    ani = animation.FuncAnimation(fig, animate, fargs=(node, line1, line2, line3, line4, ax), interval=500)
    
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()
    
    plt.show()
    
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
