#!/usr/bin/env python3

# from pynput import keyboard
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

from rclpy import qos
from src.utilize import * 
from src.controller import *
# from src.main_run import mainRun
import time 
import math


class Cmd_vel_to_motor_speed(Node):


    moveSpeed: float = 0.0
    slideSpeed: float = 0.0
    turnSpeed: float = 0.0

    # maxSpeed : int = 1023.0/2 # pwm
    # max_linear_speed = 2.0  # m/s max
    motor1Speed : float = 0
    motor2Speed : float = 0
    motor3Speed : float = 0
    motor4Speed : float = 0


    def __init__(self):
        super().__init__("Cmd_vel_to_motor_speed")

        # self.declare_parameter("mode", 1)  # ค่าเริ่มต้นคือ 1
        # self.mode = self.get_parameter("mode").get_parameter_value().integer_value
        self.mode = 1
        
        self.moveSpeed: float = 0.0
        self.slideSpeed: float = 0.0
        self.turnSpeed: float = 0.0

        self.maxSpeed : int = 1023.0 # pwm
        self.maxRPM : int = 6000
        self.max_linear_speed = 3.0  # m/s max
        self.motor1Speed : float = 0
        self.motor2Speed : float = 0
        self.motor3Speed : float = 0
        self.motor4Speed : float = 0

        self.motorshooter1Speed : float = 0
        self.motorshooter2Speed : float = 0
        self.motorshooter3Speed : float = 0
        self.yaw : float = 0
        self.yaw_setpoint = self.yaw
        
        # self.main_run_instance = mainRun()
        # self.middlecam = self.main_run_instance.center_x
        self.middlecam : float = 0.0
        
        self.macro_active = False
        self.previous_manual_turn = time.time()

        self.controller = Controller()
        self.hooprotage = Controller()

        self.hoop_distance_x : float = 0.0
        self.hoop_distance_y : float = 0.0
        
        
        self.send_robot_speed = self.create_publisher(
            Twist, "/shaq/cmd_move/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.send_shoot_speed = self.create_publisher(
            Twist, "/shaq/cmd_shoot/rpm", qos_profile=qos.qos_profile_system_default
        )


        self.create_subscription(
            Twist, '/shaq/cmd_move', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/shaq/cmd_shoot', self.cmd_shoot, qos_profile=qos.qos_profile_sensor_data # 10
        )
        
        self.create_subscription(
            Twist, '/shaq/cmd_macro', self.cmd_macro, qos_profile=qos.qos_profile_sensor_data # 10
        )

        self.create_subscription(
            Twist, '/shaq/imu/pos_angle', self.get_robot_angle, qos_profile=qos.qos_profile_sensor_data # 10
        )

        self.create_subscription(
            Float32MultiArray, '/shaq/pid/rotate', self.get_pid, qos_profile=qos.qos_profile_sensor_data # 10
        )


        self.create_subscription(
            Twist, '/shaq/send_where_hoop', self.wherehoop, qos_profile=qos.qos_profile_sensor_data # 10
        )


        self.sent_data_timer = self.create_timer(0.01, self.sendData)
        
    def wherehoop(self, msg):
        self.hoop_distance_x = msg.linear.x
        self.hoop_distance_y = msg.linear.y
        self.middlecam = msg.angular.x
        # self.middlecam = msg.angular.y

    def get_robot_angle(self,msg):
        self.yaw = WrapRads(To_Radians(msg.angular.x))
    
    def get_pid(self,msg):
        self.controller.ConfigPIDF(kp = msg.data[0], ki= msg.data[1], kd=msg.data[2], kf=msg.data[3]) 


    def cmd_vel(self, msg):

        CurrentTime = time.time()
        self.moveSpeed = msg.linear.y  
        self.slideSpeed = msg.linear.x  
        self.turnSpeed = msg.angular.z 

        # self.mode = self.get_parameter("mode").get_parameter_value().integer_value


        if self.mode == 1:
            rotation = self.controller.Calculate(WrapRads(self.yaw_setpoint - self.yaw)) 
        elif self.mode == 2:
            # rotation = self.controller.Calculate(self.hooprotage - self.main_run_instance.center_x)
            rotation = self.hooprotage.Calculate(self.hoop_distance_x - self.middlecam)

        if self.turnSpeed != 0 or (CurrentTime - self.previous_manual_turn < 0.45):
            rotation = self.turnSpeed
            self.yaw_setpoint = self.yaw

        if self.slideSpeed == 0 and self.moveSpeed  == 0 and self.turnSpeed == 0 and abs(rotation) < 0.2:
            rotation = 0

        self.previous_manual_turn = CurrentTime if self.turnSpeed != 0 else self.previous_manual_turn


        D = max(abs(self.moveSpeed)+abs(self.slideSpeed)+abs(rotation), 1.0)
        self.motor1Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed + rotation) / D * self.maxSpeed))
        self.motor2Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed - rotation) / D * self.maxSpeed))
        self.motor3Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed + rotation) / D * self.maxSpeed))
        self.motor4Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed - rotation) / D * self.maxSpeed))
        

    def cmd_shoot(self, msg):
            if not self.macro_active:  # Only update if macro is inactive
                self.motorshooter1Speed = abs(msg.linear.x - 1) * self.maxRPM 
                self.motorshooter2Speed = abs(msg.linear.x - 1) * self.maxRPM
            
            self.motorshooter3Speed = abs(msg.linear.z - 1) * self.maxSpeed
            self.motorshooter3Speed += msg.angular.x * self.maxSpeed

            if self.motorshooter3Speed >= 1023.0:
                self.motorshooter3Speed = 1023.0


    def cmd_macro(self, msg):

        if msg.linear.y == 1 :
            self.mode = 2
            

        if msg.linear.x == 1 :
            
            self.macro_active = True
            self.motorshooter1Speed = -1500.0    #Upper
            self.motorshooter2Speed = 4000.0     #Lower
         
        else:
            self.macro_active = False
         
            
    def sendData(self):
        motorspeed_msg = Twist()
        motorshooter_msg = Twist()
       
        motorspeed_msg.linear.x = float(self.motor1Speed)
        motorspeed_msg.linear.y = float(self.motor2Speed)
        motorspeed_msg.angular.x = float(self.motor3Speed)
        motorspeed_msg.angular.y = float(self.motor4Speed)

        motorshooter_msg.linear.x = float(self.motorshooter1Speed)
        motorshooter_msg.linear.y = float(self.motorshooter2Speed)
        motorshooter_msg.linear.z = float(self.motorshooter3Speed)

        motorshooter_msg.angular.x = float(self.mode)


        self.send_shoot_speed.publish(motorshooter_msg)
        self.send_robot_speed.publish(motorspeed_msg)


def main():
    rclpy.init()

    sub = Cmd_vel_to_motor_speed()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()