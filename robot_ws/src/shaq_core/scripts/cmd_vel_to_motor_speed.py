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
import time 

class Cmd_vel_to_motor_speed(Node):


    moveSpeed: float = 0.0
    slideSpeed: float = 0.0
    turnSpeed: float = 0.0

    maxSpeed : int = 1023.0/2 # pwm
    max_linear_speed = 2.0  # m/s max
    motor1Speed : float = 0
    motor2Speed : float = 0
    motor3Speed : float = 0
    motor4Speed : float = 0


    def __init__(self):
        super().__init__("Cmd_vel_to_motor_speed")
        
        self.declare_parameter("motor1", True)
        self.declare_parameter("motor2", True)
        self.declare_parameter("motor3", True)
        self.declare_parameter("motor4", True)
        
        self.motor1_enabled = self.get_parameter("motor1").get_parameter_value().bool_value
        self.motor2_enabled = self.get_parameter("motor2").get_parameter_value().bool_value
        self.motor3_enabled = self.get_parameter("motor3").get_parameter_value().bool_value
        self.motor4_enabled = self.get_parameter("motor4").get_parameter_value().bool_value

        
        self.moveSpeed: float = 0.0
        self.slideSpeed: float = 0.0
        self.turnSpeed: float = 0.0

        self.maxSpeed : int = 1023.0 # pwm
        self.max_linear_speed = 3.0  # m/s max
        self.motor1Speed : float = 0
        self.motor2Speed : float = 0
        self.motor3Speed : float = 0
        self.motor4Speed : float = 0

        self.motorshooter1Speed : float = 0
        self.motorshooter2Speed : float = 0
        self.motorshooter3Speed : float = 0
        self.lift : float = 0
        
 
        
        self.send_robot_speed = self.create_publisher(
            Twist, "shaq/cmd_vel/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.send_shoot_speed = self.create_publisher(
            Twist, "shaq/shooter/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, 'shaq/cmd_vel', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/shaq/shooter_power', self.cmd_shoot, qos_profile=qos.qos_profile_sensor_data # 10
        )


        self.sent_data_timer = self.create_timer(0.01, self.sendData)
        
    def cmd_vel(self, msg):

        CurrentTime = time.time()
        self.moveSpeed = msg.linear.y  #swap
        self.slideSpeed = msg.linear.x  #swap
        r = self.turnSpeed = msg.angular.z 
                
        D = max(abs(self.moveSpeed)+abs(self.slideSpeed)+abs(r), 1.0)
        self.motor1Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed + r) / D * self.maxSpeed))
        self.motor2Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed - r) / D * self.maxSpeed))
        self.motor3Speed = float("{:.1f}".format((self.moveSpeed - self.slideSpeed + r) / D * self.maxSpeed))
        self.motor4Speed = float("{:.1f}".format((self.moveSpeed + self.slideSpeed - r) / D * self.maxSpeed))
        

    def cmd_shoot(self, msg):

        CurrentTime = time.time()
        self.motorshooter1Speed = abs(msg.linear.x - 1) * self.maxSpeed  
        self.motorshooter2Speed = abs(msg.linear.x - 1) * self.maxSpeed
        # self.motorshooter2Speed = (abs(msg.linear.x - 1) * self.maxSpeed - 100) if (abs(msg.linear.x - 1) * self.maxSpeed > 500.0) else abs(msg.linear.x - 1) * self.maxSpeed
        self.motorshooter3Speed = (msg.linear.z * self.maxSpeed)
    
        # if msg.linear.z == 1:
        #     self.motorshooter3Speed = msg.linear.z * self.maxSpeed
        # elif msg.linear.z == -1:
        #     self.motorshooter3Speed = (msg.linear.z * self.maxSpeed) / 2
        # else:
        #     self.motorshooter3Speed = 0

        

        # self.motorshooter3Speed = max(abs(msg.linear.y - 1) * self.maxSpeed, (msg.linear.z * self.maxSpeed) / 2)
        # self.motorshooter3Speed = abs(msg.linear.y - 1) * self.maxSpeed


        # self.lift = (msg.linear.z * self.maxSpeed)/2

            
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
        # motorshooter_msg.angular.x = float(self.lift)


        self.send_shoot_speed.publish(motorshooter_msg)
        self.send_robot_speed.publish(motorspeed_msg)



def main():
    rclpy.init()

    sub = Cmd_vel_to_motor_speed()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
