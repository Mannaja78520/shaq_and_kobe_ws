#!/usr/bin/env python3


import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy import qos

from src.utilize import * 
from src.controller import *

import time 
import math


class Cmd_vel_to_motor_speed(Node):


    moveSpeed: float = 0.0
    slideSpeed: float = 0.0
    turnSpeed: float = 0.0

    motor1Speed : float = 0
    motor2Speed : float = 0
    motor3Speed : float = 0
    motor4Speed : float = 0


    def __init__(self):
        super().__init__("Cmd_vel_to_motor_speed")

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


        self.middlecam : float = 0.0
        
        self.macro_active = False
        self.previous_manual_turn = time.time()

        self.controller = Controller(kp = 1.27, ki = 0.2, kd = 0.1, errorTolerance=(To_Radians(0.5)), i_min= -1, i_max= 1)
        self.hooprotage = Controller(kp = 0.001, ki = 0.001, kd = 0.0,  errorTolerance=(5))
        self.april_controller = Controller(kp = 0.002, ki = 0.001, kd = 0.0, errorTolerance  = (10))

        self.hoop_distance_x : float = 0.0
        self.hoop_distance_y : float = 0.0

        self.middlecam_apriltag : float = 0.0
        self.distance_to_kobe : float = 0.0
        self.apriltag_distance : float = 0.0
        self.tag_id : float = 0.0

        self.servo_angle : float = 0.0

        self.encoder_mode : float = 0.0



        
        
        self.send_robot_speed = self.create_publisher(
            Twist, "/shaq/cmd_move/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.send_shoot_speed = self.create_publisher(
            Twist, "/shaq/cmd_shoot/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.send_servo_angle = self.create_publisher(
            Twist, "/shaq/cmd_servo/angle", qos_profile=qos.qos_profile_system_default
        )


        self.create_subscription(
            Twist, '/shaq/cmd_move', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/shaq/cmd_shoot', self.cmd_shoot, qos_profile=qos.qos_profile_sensor_data # 10
        )

        self.create_subscription(
            Twist, '/shaq/cmd_servo', self.cmd_servo, qos_profile=qos.qos_profile_system_default
        )
        
        self.create_subscription(
            Twist, '/shaq/cmd_macro', self.cmd_macro, qos_profile=qos.qos_profile_sensor_data # 10
        )

        # self.create_subscription(
        #     Twist, '/shaq/imu/pos_angle', self.get_robot_angle, qos_profile=qos.qos_profile_sensor_data # 10
        # )

        # self.create_subscription(
        #     Float32MultiArray, '/shaq/pid/rotate', self.get_pid, qos_profile=qos.qos_profile_sensor_data # 10
        # )


        self.create_subscription(
            Twist, '/shaq/send_where_hoop', self.wherehoop, qos_profile=qos.qos_profile_sensor_data # 10
        )

        self.create_subscription(
            Twist, '/shaq/distance/kobe', self.distance, qos_profile=qos.qos_profile_sensor_data # 10
        )

        self.create_subscription(
            Twist, "/shaq/cmd_encoder", self.encoder, qos_profile=qos.qos_profile_sensor_data
        )


        self.sent_data_timer = self.create_timer(0.03, self.sendData)

    def encoder(self,msg):
        self.encoder_mode = msg.linear.x    # 1 Bit, 2 RPM
        
    def distance(self,msg):
        self.apriltag_distance = msg.linear.x
        self.distance_to_kobe = msg.linear.z
        self.middlecam_apriltag = msg.angular.x
        self.tag_id = msg.angular.z
        


    def wherehoop(self, msg):
        self.hoop_distance_x = msg.linear.x
        self.middlecam = msg.angular.x


    def get_robot_angle(self,msg):
        self.yaw = WrapRads(To_Radians(msg.angular.x))
    
    def get_pid(self,msg):
        self.controller.ConfigPIDF(kp = msg.data[0], ki= msg.data[1], kd=msg.data[2], kf=msg.data[3]) 


    def cmd_vel(self, msg):

        CurrentTime = time.time()   
        self.moveSpeed = msg.linear.y  
        self.slideSpeed = msg.linear.x  
        self.turnSpeed = msg.angular.z 



        if self.mode == 1:
            rotation = self.controller.Calculate(WrapRads(self.yaw_setpoint - self.yaw)) 
            if self.slideSpeed == 0 and self.moveSpeed  == 0 and self.turnSpeed == 0 and abs(rotation) < 0.2:
                rotation = 0

        if self.mode == 2:
            
            rotation = self.hooprotage.Calculate(self.hoop_distance_x - self.middlecam)

            if self.hoop_distance_x == 0:
                rotation = 0.0

            error = (self.hoop_distance_x - self.middlecam)

            if abs(error) < 10:  # Error threshold for small adjustments
                boost_factor = 5.0  # Increase power by 500%
                rotation *= boost_factor
            elif abs(error) < 25:  # Error threshold for small adjustments
                boost_factor = 2.0  # Increase power by 200%
                rotation *= boost_factor
            elif abs(error) < 70:  # Error threshold for small adjustments
                boost_factor = 2.0  # Increase power by 20%
                rotation *= boost_factor

        if self.mode == 3:
            rotation = self.april_controller.Calculate(self.apriltag_distance - self.middlecam_apriltag)

            if self.apriltag_distance == 0:
                rotation = 0.0

            error = (self.apriltag_distance - self.middlecam_apriltag)

            if abs(error) < 10:  # Error threshold for small adjustments
                boost_factor = 5.0  # Increase power by 500%
                rotation *= boost_factor
            elif abs(error) < 25:  # Error threshold for small adjustments
                boost_factor = 2.0  # Increase power by 200%
                rotation *= boost_factor
            elif abs(error) < 40:  # Error threshold for small adjustments
                boost_factor = 3.0  # Increase power by 20%
                rotation *= boost_factor
   
    

        if self.turnSpeed != 0 or (CurrentTime - self.previous_manual_turn < 0.45):
            rotation = self.turnSpeed
            self.yaw_setpoint = self.yaw


        self.previous_manual_turn = CurrentTime if self.turnSpeed != 0 else self.previous_manual_turn


        D = max(abs(self.moveSpeed)+abs(self.slideSpeed)+abs(rotation), 1.0)
        self.motor1Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed + rotation) / D * self.maxSpeed))
        self.motor2Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed - rotation) / D * self.maxSpeed))
        self.motor3Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed + rotation) / D * self.maxSpeed))
        self.motor4Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed - rotation) / D * self.maxSpeed))
        

    def cmd_shoot(self, msg):
            if not self.macro_active:  # Only update if macro is inactive
                if msg.linear.x == 1 and msg.linear.y == 1:
                    self.motorshooter1Speed = 0.0
                    self.motorshooter2Speed = 0.0
                else:
                    self.motorshooter1Speed = min((abs(msg.linear.x - 1)) * self.maxSpeed + 500, self.maxSpeed)
                    self.motorshooter2Speed = min((abs(msg.linear.x - 1)) * self.maxSpeed + 500, self.maxSpeed)
                
            self.motorshooter3Speed = abs(msg.linear.z - 1) * self.maxSpeed
            self.motorshooter3Speed += msg.angular.x * self.maxSpeed

            if self.motorshooter3Speed >= 1023.0:
                self.motorshooter3Speed = 1023.0


    def cmd_macro(self, msg):

        if self.encoder_mode == 1:      #1 Bit

            if msg.linear.z == 1: #Shoot    #Cross
                self.macro_active = True
                self.motorshooter1Speed = 750.0  # Upper
                self.motorshooter2Speed = 770.0  # Lower


            elif msg.linear.x == 1: #Dribble    #Triangle
                self.macro_active = True
                self.motorshooter1Speed = -610.0  # Upper
                self.motorshooter2Speed = 700.0   # Lower


            elif msg.angular.y == 1: #Pass      #L1
                self.macro_active = True
                self.motorshooter1Speed = 635.0  # Upper
                self.motorshooter2Speed = 635.0   # Lower
            
            elif msg.angular.z == 1: #Shoot2    #R1
                self.macro_active = True
                self.motorshooter1Speed = 710.0  # Upper
                self.motorshooter2Speed = 710.0   # Lower
        
            else:
                self.macro_active = False 

        else:   #RPM

            if msg.linear.z == 1: #Shoot    #Cross
                self.macro_active = True
                self.motorshooter1Speed = 4700.0  # Upper
                self.motorshooter2Speed = 5400.0  # Lower


            elif msg.linear.x == 1: #Dribble    #Triangle
                self.macro_active = True
                self.motorshooter1Speed = -2100.0  # Upper
                self.motorshooter2Speed = 5300.0   # Lower


            elif msg.angular.y == 1: #Pass      #L1
                self.macro_active = True
                self.motorshooter1Speed = 550.0  # Upper
                self.motorshooter2Speed = 550.0   # Lower
            
            elif msg.angular.z == 1: #Shoot2    #R1
                self.macro_active = True
                self.motorshooter1Speed = 4300.0  # Upper
                self.motorshooter2Speed = 4700.0   # Lower
        
            else:
                self.macro_active = False 


        if msg.linear.y == 1 :      # Circle
            self.mode = 2

        elif msg.angular.x == 1:    # Square
            self.mode = 3
            
        else:
            self.mode = 1           # Default
            
    
    def cmd_servo(self, msg):
        
        if msg.linear.x == 1:               # Closed Servo
            self.servo_angle = float(0.0)

        if msg.linear.x == 2:               # Opened Servo
            self.servo_angle = float(60.0)
            
    def sendData(self):
        motorspeed_msg = Twist()
        motorshooter_msg = Twist()
        servo_msg = Twist()
       
        motorspeed_msg.linear.x = float(self.motor1Speed)
        motorspeed_msg.linear.y = float(self.motor2Speed)
        motorspeed_msg.angular.x = float(self.motor3Speed)
        motorspeed_msg.angular.y = float(self.motor4Speed)

        motorshooter_msg.linear.x = float(self.motorshooter1Speed)
        motorshooter_msg.linear.y = float(self.motorshooter2Speed)
        motorshooter_msg.linear.z = float(self.motorshooter3Speed)

        motorshooter_msg.angular.x = float(self.mode)
        motorshooter_msg._angular.y = float(self.encoder_mode)

        servo_msg.linear.x = float(self.servo_angle)

        self.send_servo_angle.publish(servo_msg)
        self.send_shoot_speed.publish(motorshooter_msg)
        self.send_robot_speed.publish(motorspeed_msg)


def main():
    rclpy.init()

    sub = Cmd_vel_to_motor_speed()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()