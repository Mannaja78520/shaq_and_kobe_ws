#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Change this based on your message type
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time

class EncoderSubscriber(Node):
    def __init__(self):
        super().__init__('encoder_sub')
        self.subscription = self.create_subscription(
            Float32,  
            'robot1/rpm', 
            self.listener_callback,
            10)
        self.subscription
        
        self.times = []
        self.values = []
        self.start_time = time.time()

    def listener_callback(self, msg):
        current_time = time.time() - self.start_time
        self.times.append(current_time)
        self.values.append(msg.data)
        self.get_logger().info(f'Received: {msg.data} at {current_time:.2f} seconds')

    def get_data(self):
        return self.times, self.values


def animate(i, node, line):
    times, values = node.get_data()
    line.set_xdata(times)
    line.set_ydata(values)
    plt.xlim(max(0, times[-1] - 10), times[-1] + 1)  #10 sec
    plt.ylim(min(values) - 1, max(values) + 1) if values else plt.ylim(-1, 1)
    return line,


def main(args=None):
    rclpy.init(args=args)
    node = EncoderSubscriber()
    
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'r-', label='Encoder Value')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Value')
    ax.legend()
    
    ani = animation.FuncAnimation(fig, animate, fargs=(node, line), interval=500)
    plt.show()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
