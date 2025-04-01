#!/usr/bin/env python3

# from pynput import keyboard
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy import qos
from kobe_core.utilize import *
from kobe_core.controller import *
import time 

class Cmd_vel_to_motor_speed(Node):

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
        self.turnSpeed: float = 0.0

        self.maxSpeed : float = 1023.0
        self.motor1Speed : float = 0
        self.motor2Speed : float = 0
        
        self.trackWidth : float = 2.0

        self.yaw : float = 0
        self.yaw_setpoint = self.yaw

        self.previous_manual_turn = time.time()

        self.controller = Controller(kp = 2.5, ki = 0.05, kd = 0.001, errorTolerance= To_Radians(0.5), i_min= -1, i_max= 1)
        
        

        self.motorshooter1Speed : float = 0
        self.motorshooter2Speed : float = 0
        self.motorshooter3Speed : float = 0

        self.motornadeem : float = 0
        
        
        self.macro_active = False
        
 
        
        self.send_robot_speed = self.create_publisher(
            Twist, "/kobe/cmd_move/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.send_shoot_speed = self.create_publisher(
            Twist, "/kobe/cmd_shoot/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.send_nadeem_speed = self.create_publisher(
            Twist, "/kobe/cmd_nadeem/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/kobe/cmd_move', self.cmd_move, qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/kobe/cmd_shoot', self.cmd_shoot, qos_profile=qos.qos_profile_sensor_data # 10
        )
        
        self.create_subscription(
            Twist, '/kobe/cmd_macro', self.cmd_macro, qos_profile=qos.qos_profile_sensor_data # 10
        )
        self.create_subscription(
            Twist, '/kobe/cmd_nadeem', self.cmd_nadeem, qos_profile=qos.qos_profile_sensor_data #10
        )
        
        self.create_subscription(
              Float32MultiArray, '/kobe/pid/rotate', self.get_pid, qos_profile=qos.qos_profile_sensor_data # 10
          )
        
        self.create_subscription(
            Twist, '/kobe/imu/pos_angle', self.get_robot_angle, qos_profile=qos.qos_profile_sensor_data # 10
        )



        self.sent_data_timer = self.create_timer(0.01, self.sendData)
        

    def get_robot_angle(self,msg):
        self.yaw = WrapRads(To_Radians(msg.angular.x) * -1)

    def get_pid(self,msg):
        self.controller.ConfigPIDF(kp = msg.data[0], ki= msg.data[1], kd=msg.data[2], kf=msg.data[3]) 


    def cmd_move(self, msg):

        
        # Initialize CurrentTime at the start
        CurrentTime = time.time()

        # Handle turning behavior and set the yaw setpoint
        if self.turnSpeed != 0 or (CurrentTime - self.previous_manual_turn < 0.45):
            rotation = self.turnSpeed
            self.yaw_setpoint = self.yaw
        else:
            # Apply PID controller based on yaw setpoint
            rotation = self.controller.Calculate(WrapRads(self.yaw_setpoint - self.yaw))

        # If no movement or turn, stop rotation (ignoring slideSpeed logic)
        if self.moveSpeed == 0 and self.turnSpeed == 0 :
            rotation = 0
            self.yaw_setpoint = self.yaw

        # Update the previous manual turn time
        self.previous_manual_turn = CurrentTime if self.turnSpeed != 0 else self.previous_manual_turn

        # Set move and turn speeds from the message input
        self.moveSpeed = msg.linear.x

        self.turnSpeed = msg.angular.x 

        # Calculate motor speeds based on move and turn speeds
        self.motor1Speed = (self.moveSpeed + rotation) * self.maxSpeed 
        self.motor2Speed = (self.moveSpeed - rotation) * self.maxSpeed 
        self.rotation = rotation * self.maxSpeed / 3  # Apply track width to rotation speed

        reverse = False  

        # Limit motor speeds to a maximum value
        max_motor_speed = 1023.0 / 1.5 
        self.motor1Speed = min(self.motor1Speed, max_motor_speed)
        self.motor2Speed = min(self.motor2Speed, max_motor_speed)
        self.rotation = min(self.rotation, max_motor_speed)
            
        # self.motor1Speed = max(min(self.motor1Speed, 1.0), -1.0)
        # self.motor2Speed = max(min(self.motor2Speed, 1.0), -1.0)
        
        # self.motor1Speed = self.map_speed_to_pwm(self.motor1Speed)
        
    def map_speed_to_pwm(self, speed):
        # Map motor speed [-1.0, 1.0] to PWM range [1000, 2000]
        pwm = int(speed * self.maxSpeed)  # Center at 1500ms, range from 1000ms to 2000ms
        return pwm    


    def cmd_shoot(self, msg):
        if not self.macro_active:  # Only update if macro is inactive
            self.motorshooter1Speed = abs(msg.linear.x - 1) * self.maxSpeed
            self.motorshooter2Speed = abs(msg.linear.x - 1) * self.maxSpeed
        
            self.motorshooter3Speed = abs(msg.linear.z - 1) * self.maxSpeed
            self.motorshooter3Speed += msg.angular.x * self.maxSpeed

        if self.motorshooter3Speed >= 1023.0:
            self.motorshooter3Speed = 1023.0 

    def cmd_nadeem(self, msg):
        if not self.macro_active:  # Only update if macro is inactive
            self.motornadeem = abs(msg.linear.x - 1) * self.maxSpeed
            

    def cmd_macro(self, msg):
        if msg.linear.x == 1 :
            
            self.macro_active = True
            self.motorshooter1Speed = 500.0     #Upper
            self.motorshooter2Speed = 800.0     #Lower
         
        else:
            self.macro_active = False
         
            
    def sendData(self):
        motorspeed_msg = Twist()
        motorshooter_msg = Twist()
        motornadeem_msg = Twist()

       
        motorspeed_msg.linear.x = float(self.motor1Speed)
        motorspeed_msg.linear.y = float(self.motor2Speed)


        motorshooter_msg.linear.x = float(self.motorshooter1Speed)
        motorshooter_msg.linear.y = float(self.motorshooter2Speed)
        motorshooter_msg.linear.z = float(self.motorshooter3Speed)

        motornadeem_msg.linear.x = float(self.motornadeem)
        motornadeem_msg.angular.x = float(self.yaw)


        self.send_shoot_speed.publish(motorshooter_msg)
        self.send_robot_speed.publish(motorspeed_msg)
        self.send_nadeem_speed.publish(motornadeem_msg)



def main():
    rclpy.init()

    sub = Cmd_vel_to_motor_speed()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()