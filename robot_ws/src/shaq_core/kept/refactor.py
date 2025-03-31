import time
import math
import rclpy
from rclpy.node import Node
from rclpy import qos
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from src.utilize import *
from src.controller import *

class CmdVelToMotorSpeed(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_motor_speed")
        
        self.mode = 1
        self.max_speed = 1023.0  # PWM max
        self.max_rpm = 6000
        self.max_linear_speed = 3.0  # m/s
        self.yaw = 0.0
        self.yaw_setpoint = 0.0
        self.hoop_distance_x = 0.0
        self.hoop_distance_y = 0.0
        self.middlecam = 0.0
        self.macro_active = False
        self.previous_manual_turn = time.time()

        self.controller = Controller()
        self.hoop_controller = Controller()
        
        self.init_publishers()
        self.init_subscribers()
        self.sent_data_timer = self.create_timer(0.01, self.send_data)

    def init_publishers(self):
        self.send_robot_speed = self.create_publisher(Twist, "/shaq/cmd_move/rpm", qos.qos_profile_system_default)
        self.send_shoot_speed = self.create_publisher(Twist, "/shaq/cmd_shoot/rpm", qos.qos_profile_system_default)

    def init_subscribers(self):
        self.create_subscription(Twist, '/shaq/cmd_move', self.cmd_vel, qos.qos_profile_system_default)
        self.create_subscription(Twist, '/shaq/cmd_shoot', self.cmd_shoot, qos.qos_profile_sensor_data)
        self.create_subscription(Twist, '/shaq/cmd_macro', self.cmd_macro, qos.qos_profile_sensor_data)
        self.create_subscription(Twist, '/shaq/imu/pos_angle', self.get_robot_angle, qos.qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/shaq/pid/rotate', self.get_pid, qos.qos_profile_sensor_data)
        self.create_subscription(Twist, '/shaq/send_where_hoop', self.wherehoop, qos.qos_profile_sensor_data)

    def wherehoop(self, msg):
        self.hoop_distance_x, self.hoop_distance_y, self.middlecam = msg.linear.x, msg.linear.y, msg.angular.x

    def get_robot_angle(self, msg):
        self.yaw = WrapRads(To_Radians(msg.angular.x))

    def get_pid(self, msg):
        self.controller.ConfigPIDF(kp=msg.data[0], ki=msg.data[1], kd=msg.data[2], kf=msg.data[3])

    def cmd_vel(self, msg):
        current_time = time.time()
        move_speed, slide_speed, turn_speed = msg.linear.y, msg.linear.x, msg.angular.z
        rotation = self.calculate_rotation(turn_speed, current_time)
        
        motor_speeds = self.compute_motor_speeds(move_speed, slide_speed, rotation)
        self.motor1_speed, self.motor2_speed, self.motor3_speed, self.motor4_speed = motor_speeds

    def calculate_rotation(self, turn_speed, current_time):
        if self.mode == 1:
            rotation = self.controller.Calculate(WrapRads(self.yaw_setpoint - self.yaw))
        elif self.mode == 2:
            rotation = self.hoop_controller.Calculate(self.hoop_distance_x - self.middlecam)
        
        if turn_speed != 0 or (current_time - self.previous_manual_turn < 0.45):
            rotation = turn_speed
            self.yaw_setpoint = self.yaw
        
        if abs(rotation) < 0.2:
            rotation = 0
        
        self.previous_manual_turn = current_time if turn_speed != 0 else self.previous_manual_turn
        return rotation

    def compute_motor_speeds(self, move_speed, slide_speed, rotation):
        D = max(abs(move_speed) + abs(slide_speed) + abs(rotation), 1.0)
        return (
            round((move_speed + slide_speed + rotation) / D * self.max_speed, 1),
            round((move_speed - slide_speed - rotation) / D * self.max_speed, 1),
            round((move_speed - slide_speed + rotation) / D * self.max_speed, 1),
            round((move_speed + slide_speed - rotation) / D * self.max_speed, 1)
        )

    def cmd_shoot(self, msg):
        if not self.macro_active:
            self.motorshooter1_speed = abs(msg.linear.x - 1) * self.max_rpm
            self.motorshooter2_speed = abs(msg.linear.x - 1) * self.max_rpm
        
        self.motorshooter3_speed = min(abs(msg.linear.z - 1) * self.max_speed + msg.angular.x * self.max_speed, 1023.0)

    def cmd_macro(self, msg):
        self.mode = 2 if msg.linear.y == 1 else 1
        self.macro_active = msg.linear.x == 1
        
        if self.macro_active:
            self.motorshooter1_speed, self.motorshooter2_speed = -1500.0, 4000.0
        
    def send_data(self):
        motorspeed_msg = Twist()
        motorspeed_msg.linear.x, motorspeed_msg.linear.y = self.motor1_speed, self.motor2_speed
        motorspeed_msg.angular.x, motorspeed_msg.angular.y = self.motor3_speed, self.motor4_speed

        motorshooter_msg = Twist()
        motorshooter_msg.linear.x, motorshooter_msg.linear.y, motorshooter_msg.linear.z = (
            self.motorshooter1_speed, self.motorshooter2_speed, self.motorshooter3_speed
        )
        motorshooter_msg.angular.x = float(self.mode)

        self.send_shoot_speed.publish(motorshooter_msg)
        self.send_robot_speed.publish(motorspeed_msg)


def main():
    rclpy.init()
    node = CmdVelToMotorSpeed()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
