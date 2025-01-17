#!/usr/bin/env python3

from pynput import keyboard
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy import qos
from src.utilize import * 
from src.controller import * 

class test_sub_cmd_vel(Node):
    
    
    # PID controllers for motors
    controller1 = Controller(kp=9.0, ki=0.42, kd=0.0, kf = 199.59, errorTolerance=0.01)
    controller2 = Controller(kp=7.0, ki=0.42, kd=0.0, kf = 196.2, errorTolerance=0.01)
    controller3 = Controller(kp=7.0, ki=0.42, kd=0.0, kf = 199.1, errorTolerance=0.01)
    controller4 = Controller(kp=13.0, ki=0.42, kd=0.0, kf = 234.7, errorTolerance=0.01)

    moveSpeed: float = 0.0
    slideSpeed: float = 0.0
    turnSpeed: float = 0.0

    maxSpeed : int = 1023.0 # pwm
    max_linear_speed = 3.0  # m/s max
    motor1Speed : float = 0
    motor2Speed : float = 0
    motor3Speed : float = 0
    motor4Speed : float = 0

    # encoder1RPM : float = 0
    # encoder2RPM : float = 0
    # encoder3RPM : float = 0
    # encoder4RPM : float = 0

    # encoder1MPS : float = 0
    # encoder2MPS : float = 0
    # encoder3MPS : float = 0
    # encoder4MPS : float = 0
    wheel_radians = 0.05
    gearRatio: float = 20.0 / 23.0

    motor_speeds = [0.0, 0.0, 0.0, 0.0]
    encoder_speeds = [0.0, 0.0, 0.0, 0.0]  # Speeds in m/s
    yaw:float = 0.0
    pitch:float = 0.0
    roll:float = 0.0


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
        
        self.imu_sub = self.create_subscription(
            Twist, '/debug/imu', self.imu_callback, qos_profile=qos.qos_profile_system_default
        )
        
        self.keyboard_sub = self.create_subscription(
            String, '/keyboard', self.keyboard_callback, 10
        )

        self.sent_data_timer = self.create_timer(0.01, self.sendData)

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
            
    def imu_callback(self, msg):
        self.roll = msg.angular.x
        self.pitch = msg.angular.y
        self.yaw = msg.angular.z
            
    def encoder_callback(self, msg):
        self.encoder_speeds[0] = (msg.linear.x * ((2 * 3.14 * self.wheel_radians) / 60) * self.gearRatio)   #m/s
        self.encoder_speeds[1] = (msg.linear.y * ((2 * 3.14 * self.wheel_radians) / 60) * self.gearRatio)
        self.encoder_speeds[2] = (msg.linear.z * ((2 * 3.14 * self.wheel_radians) / 60) * self.gearRatio)
        self.encoder_speeds[3] = (msg.angular.x * ((2 * 3.14 * self.wheel_radians) / 60) * self.gearRatio)
        
        wheelSpeed_msg = Twist()
        wheelSpeed_msg.linear.x = float(self.encoder_speeds[0])
        wheelSpeed_msg.linear.y = float(self.encoder_speeds[1])
        wheelSpeed_msg.linear.z = float(self.encoder_speeds[2])
        wheelSpeed_msg.angular.x = float(self.encoder_speeds[3])
        
        self.send_wheel_speed.publish(wheelSpeed_msg)
        
        
        
        # # Update encoder speeds
        # self.encoder_speeds[0] = msg.linear.x
        # self.encoder_speeds[1] = msg.linear.y
        # self.encoder_speeds[2] = msg.linear.z
        # self.encoder_speeds[3] = msg.angular.x

        # # Initialize max RPMs if not already set
        # if not hasattr(self, 'max_rpms'):
        #     self.max_rpms = [float('-inf')] * 4  # [max_rpm1, max_rpm2, max_rpm3, max_rpm4]

        # # Update max RPMs
        # self.max_rpms[0] = max(self.max_rpms[0], msg.linear.x)
        # self.max_rpms[1] = max(self.max_rpms[1], msg.linear.y)
        # self.max_rpms[2] = max(self.max_rpms[2], msg.linear.z)
        # self.max_rpms[3] = max(self.max_rpms[3], msg.angular.x)

        # # Log the current encoder speeds and max RPMs using ROS 2 logger
        # self.get_logger().info(f"Current Speeds: {self.encoder_speeds}")
        # self.get_logger().info(f"Max RPMs: {self.max_rpms}")
        
    def cmd_vel(self, msg):
        # self.moveSpeed = msg.linear.x
        # self.slideSpeed = msg.linear.y
        # self.turnSpeed = msg.angular.z
        self.move_speed = clip(msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        self.slide_speed = clip(msg.linear.y, -self.max_linear_speed, self.max_linear_speed)
        self.turn_speed = clip(msg.angular.z, -self.max_linear_speed, self.max_linear_speed)
        # print(msg.linear.x)
        
        # D = max(abs(self.moveSpeed)+abs(self.slideSpeed)+abs(self.turnSpeed), 2.0)
        # self.motor1Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed - self.turnSpeed) / D * self.maxSpeed))
        # self.motor2Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed + self.turnSpeed) / D * self.maxSpeed))
        # self.motor3Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed - self.turnSpeed) / D * self.maxSpeed))
        # self.motor4Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed + self.turnSpeed) / D * self.maxSpeed))
        
        # Normalize and calculate motor speeds
        # D = max(abs(self.move_speed) + abs(self.slide_speed) + abs(self.turn_speed), self.max_speed)
        # self.motor1Speed = self.move_speed + self.slide_speed - self.turn_speed
        # self.motor2Speed = self.move_speed + self.slide_speed + self.turn_speed
        # self.motor3Speed = self.move_speed - self.slide_speed - self.turn_speed
        # self.motor4Speed = self.move_speed - self.slide_speed + self.turn_speed

        # Calculate motor speeds directly in m/s
        self.motor_speeds[0] = self.move_speed + self.slide_speed - self.turn_speed
        self.motor_speeds[1] = self.move_speed + self.slide_speed + self.turn_speed
        self.motor_speeds[2] = self.move_speed - self.slide_speed - self.turn_speed
        self.motor_speeds[3] = self.move_speed - self.slide_speed + self.turn_speed

        # Normalize motor speeds to respect max linear speed
        max_motor_speed = max(abs(speed) for speed in self.motor_speeds)
        if max_motor_speed > self.max_linear_speed:
            self.motor_speeds = [speed / max_motor_speed * self.max_linear_speed for speed in self.motor_speeds]

        # self.motor_speeds[0] = self.motor_speeds[0] / self.max_linear_speed * self.maxSpeed
        # self.motor_speeds[1] = self.motor_speeds[1] / self.max_linear_speed * self.maxSpeed
        # self.motor_speeds[2] = self.motor_speeds[2] / self.max_linear_speed * self.maxSpeed
        # self.motor_speeds[3] = self.motor_speeds[3] / self.max_linear_speed * self.maxSpeed


        # print("motor 1 : " + str(self.motor1Speed),"motor 2 : " +  str(self.motor2Speed),"motor 3 : " +  str(self.motor3Speed),"motor 4 : " +  str(self.motor4Speed))


    def sendData(self):
        motorspeed_msg = Twist()

        # Apply PID control to motor speeds if motors are enabled
        if self.motor1_enabled:
            motorspeed_msg.linear.x = float(clip(self.controller1.CalculateWithSetpoint(self.motor_speeds[0], self.encoder_speeds[0]), -self.maxSpeed, self.maxSpeed))
            # motorspeed_msg.linear.x = float(self.motor1Speed)
        if self.motor2_enabled:
            motorspeed_msg.linear.y = float(clip(self.controller2.CalculateWithSetpoint(self.motor_speeds[1], self.encoder_speeds[1]), -self.maxSpeed, self.maxSpeed))
            # motorspeed_msg.linear.y = float(self.motor2Speed)
        if self.motor3_enabled:
            motorspeed_msg.linear.z = float(clip(self.controller3.CalculateWithSetpoint(self.motor_speeds[2], self.encoder_speeds[2]), -self.maxSpeed, self.maxSpeed))
            # motorspeed_msg.linear.z = float(self.motor3Speed)
        if self.motor4_enabled:
            motorspeed_msg.angular.x = float(clip(self.controller4.CalculateWithSetpoint(self.motor_speeds[3], self.encoder_speeds[3]), -self.maxSpeed, self.maxSpeed))
            # motorspeed_msg.angular.x = float(self.motor4Speed)

        self.send_robot_speed.publish(motorspeed_msg)

def main():
    rclpy.init()

    sub = test_sub_cmd_vel()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
