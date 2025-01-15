#!/usr/bin/env python3

from pynput import keyboard
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from rclpy import qos
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, DurabilityPolicy
from src.utilize import * 
from src.controller import * 

import time

class test_sub_cmd_vel(Node):


    def __init__(self):
        super().__init__("Test_sub_cmd_vel_Node")
        
        self.declare_parameter("motor1", True)
        self.declare_parameter("motor2", True)
        self.declare_parameter("motor3", True)
        self.declare_parameter("motor4", True)
        
        self.motor1_enabled = self.get_parameter("motor1").get_parameter_value().bool_value
        self.motor2_enabled = self.get_parameter("motor2").get_parameter_value().bool_value
        self.motor3_enabled = self.get_parameter("motor3").get_parameter_value().bool_value
        self.motor4_enabled = self.get_parameter("motor4").get_parameter_value().bool_value

        # PID controllers for motors
        self.controller1 = Controller(kp=9.0, ki=0.42, kd=0.0, kf = 199.59, errorTolerance=0.01)
        self.controller2 = Controller(kp=7.0, ki=0.42, kd=0.0, kf = 196.2, errorTolerance=0.01)
        self.controller3 = Controller(kp=7.0, ki=0.42, kd=0.0, kf = 199.1, errorTolerance=0.01)
        self.controller4 = Controller(kp=13.0, ki=0.42, kd=0.0, kf = 234.7, errorTolerance=0.01)
        self.controllerRotageYawRobot = Controller(kp=2.32, ki=0.1, kd=0.0, errorTolerance=To_Radians(0.5), i_max=1)

        self.moveSpeed: float = 0.0
        self.slideSpeed: float = 0.0
        self.turnSpeed: float = 0.0

        self.maxSpeed : int = 1023.0*1.0/2.0 # pwm
        self.max_linear_speed = 3.0  # m/s max
        self.motor1Speed : float = 0
        self.motor2Speed : float = 0
        self.motor3Speed : float = 0
        self.motor4Speed : float = 0

        self.wheel_radians = 0.05
        self.gearRatio: float = 20.0 / 23.0

        self.motor_speeds = [0.0, 0.0, 0.0, 0.0]
        self.encoder_speeds = [0.0, 0.0, 0.0, 0.0]  # Speeds in m/s
        
        self.lastTurnTime = time.time()
        self.setpoint = 0
        self.yaw:float = 0.0
        self.pitch:float = 0.0
        self.roll:float = 0.0
        
        self.send_robot_speed = self.create_publisher(
            Twist, "/motor_speed", qos_profile=qos.qos_profile_system_default
        )
        
        self.send_wheel_speed = self.create_publisher(
            Twist, "/wheel_mps", qos_profile=qos.qos_profile_system_default
        )

        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )

        self.encoder_sub = self.create_subscription(
            Twist, '/debug/move_encoder', self.encoder_callback, qos_profile=qos.qos_profile_system_default
        )
        
        self.imu_pos_angle_sub = self.create_subscription(
            Twist, 'imu/pos_angle', self.imu_pos_angle_callback, qos_profile=qos.qos_profile_system_default
        )
        
        self.imu_data_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_data_callback, qos_profile=qos.qos_profile_sensor_data
        )
        
        self.keyboard_sub = self.create_subscription(
            String, '/keyboard', self.keyboard_callback, 10
        )

        self.sent_data_timer = self.create_timer(0.03, self.sendData)

    def keyboard_callback(self, msg):
        if msg.data == '1':
            self.motor1_enabled = True
            self.motor2_enabled = False
            self.motor3_enabled = False
            self.motor4_enabled = False
        elif msg.data == '2':
            self.motor1_enabled = False
            self.motor2_enabled = True
            self.motor3_enabled = False
            self.motor4_enabled = False
        elif msg.data == '3':
            self.motor1_enabled = False
            self.motor2_enabled = False
            self.motor3_enabled = True
            self.motor4_enabled = False
        elif msg.data == '4':
            self.motor1_enabled = False
            self.motor2_enabled = False
            self.motor3_enabled = False
            self.motor4_enabled = True
        elif msg.data == 'f':
            self.motor1_enabled = True
            self.motor2_enabled = True
            self.motor3_enabled = True
            self.motor4_enabled = True
            
    def imu_pos_angle_callback(self, msg):
        self.roll = WrapRads(To_Radians(msg.angular.x))
        self.pitch = WrapRads(To_Radians(msg.angular.y))
        self.yaw = WrapRads(To_Radians(msg.angular.z))
        
        # self.get_logger().info(f"IMU Update - Yaw: {self.yaw}, Pitch: {self.pitch}, Roll: {self.roll}")
            
    def imu_data_callback(self, msg):
        linear_accel_covariance = msg.linear_acceleration_covariance
        angular_velocity_covariance = msg.angular_velocity_covariance
        orientation_covariance = msg.orientation_covariance

        self.get_logger().debug(
            f"Covariance Matrices:\n"
            f"Linear Acceleration: {linear_accel_covariance}\n"
            f"Angular Velocity: {angular_velocity_covariance}\n"
            f"Orientation: {orientation_covariance}"
        )

        # Additional processing can go here
        pass
    
    def encoder_callback(self, msg):
        self.encoder_speeds[0] = (msg.linear.x  * ((2 * 3.14 * self.wheel_radians) / 60) * self.gearRatio)   #m/s
        self.encoder_speeds[1] = (msg.linear.y  * ((2 * 3.14 * self.wheel_radians) / 60) * self.gearRatio)
        self.encoder_speeds[2] = (msg.linear.z  * ((2 * 3.14 * self.wheel_radians) / 60) * self.gearRatio)
        self.encoder_speeds[3] = (msg.angular.x * ((2 * 3.14 * self.wheel_radians) / 60) * self.gearRatio)
        
        wheelSpeed_msg = Twist()
        wheelSpeed_msg.linear.x = float(self.encoder_speeds[0])
        wheelSpeed_msg.linear.y = float(self.encoder_speeds[1])
        wheelSpeed_msg.linear.z = float(self.encoder_speeds[2])
        wheelSpeed_msg.angular.x = float(self.encoder_speeds[3])
        
        self.send_wheel_speed.publish(wheelSpeed_msg)
        
    def cmd_vel(self, msg):
        CurrentTime = time.time()
        self.moveSpeed = msg.linear.x * 0.45
        self.slideSpeed = msg.linear.y * 0.0
        r = self.turnSpeed = msg.angular.z * 0.35
        
        # r = 0
        
        # if self.turnSpeed != 0:
        #     r = self.turnSpeed
        #     self.lastTurnTime = time.time()
        #     self.setpoint = self.yaw
        
        # if (CurrentTime - self.lastTurnTime) < 0.35 or (self.moveSpeed == 0 and self.slideSpeed == 0 and self.turnSpeed == 0):
        #     self.setpoint = self.yaw
        
        # if (self.moveSpeed != 0 or self.slideSpeed != 0) and self.turnSpeed == 0:
        #     r = self.controllerRotageYawRobot.Calculate(WrapDegs(self.setpoint - self.yaw))
        
        # if self.moveSpeed == 0 and self.slideSpeed == 0 and self.turnSpeed == 0:
        #     self.setpoint = self.yaw
            
        # r = self.controllerRotageYawRobot.Calculate(WrapRads(self.setpoint - self.yaw))   
        # if (self.moveSpeed == 0.0 and self.slideSpeed == 0.0 and self.turnSpeed == 0.0) and abs(r) < 0.035:
        #     r = 0.0
        
        # self.lastTurnTime = CurrentTime if self.turnSpeed != 0 else self.lastTurnTime
        # if (self.turnSpeed != 0 or  CurrentTime - self.lastTurnTime < 0.45) :
        #     r = self.turnSpeed
        #     self.setpoint = self.yaw
                
        D = max(abs(self.moveSpeed)+abs(self.slideSpeed)+abs(r), 1.0)
        self.motor1Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed - r) / D * self.maxSpeed))
        self.motor2Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed + r) / D * self.maxSpeed))
        self.motor3Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed - r) / D * self.maxSpeed))
        self.motor4Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed + r) / D * self.maxSpeed))
        # self.motor1Speed = round((self.moveSpeed + self.slideSpeed - r) / D * self.maxSpeed, 1)
        # self.motor2Speed = round((self.moveSpeed + self.slideSpeed + r) / D * self.maxSpeed, 1)
        # self.motor3Speed = round((self.moveSpeed - self.slideSpeed - r) / D * self.maxSpeed, 1)
        # self.motor4Speed = round((self.moveSpeed - self.slideSpeed + r) / D * self.maxSpeed, 1)
        # print(r)
        
        
        

    def sendData(self):
        motorspeed_msg = Twist()

        # Apply PID control to motor speeds if motors are enabled
        if self.motor1_enabled:
            motorspeed_msg.linear.x = float(self.motor1Speed)
        if self.motor2_enabled:
            motorspeed_msg.linear.y = float(self.motor2Speed)
        if self.motor3_enabled:
            motorspeed_msg.linear.z = float(self.motor3Speed)
        if self.motor4_enabled:
            motorspeed_msg.angular.x = float(self.motor4Speed)

        # motorspeed_msg.angular.z = float(self.yaw)
        
        self.send_robot_speed.publish(motorspeed_msg)

def main():
    rclpy.init()

    sub = test_sub_cmd_vel()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
