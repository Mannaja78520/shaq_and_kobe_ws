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
        
        self.mode = 1
        
        self.moveSpeed: float = 0.0
        self.turnSpeed: float = 0.0

        self.maxSpeed : float = 1023.0
        self.shootmaxSpeed : float = 1023.0
        self.motor1Speed : float = 0
        self.motor2Speed : float = 0
        
        self.trackWidth : float = 2.0

        self.yaw : float = 0
        self.yaw_setpoint = self.yaw

        self.previous_manual_turn = time.time()

        self.controller = Controller(kp = 1.0, ki = 0.05, kd = 0.001, baseSpeed = 0.3  ,errorTolerance= To_Radians(0.5), i_min= -1, i_max= 1)
        self.hooprotage = Controller(kp = 0.001, ki = 0.001, kd = 0.0,  errorTolerance=(5))
        self.april_controller = Controller(kp = 0.002, ki = 0.001, kd = 0.0, errorTolerance  = (10))
        

        self.motorshooter1Speed : float = 0
        self.motorshooter2Speed : float = 0
        self.motorshooter3Speed : float = 0

            
        self.macro_active = False

        self.middlecam : float = 0.0

        self.hoop_distance_x : float = 0.0
        self.hoop_distance_y : float = 0.0

        self.middlecam_apriltag : float = 0.0
        self.distance_to_kobe : float = 0.0
        self.apriltag_distance : float = 0.0
        self.tag_id : float = 0.0



        
 
        
        self.send_robot_speed = self.create_publisher(
            Twist, "/kobe/cmd_move/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.send_shoot_speed = self.create_publisher(
            Twist, "/kobe/cmd_shoot/rpm", qos_profile=qos.qos_profile_system_default
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
              Float32MultiArray, '/kobe/pid/rotate', self.get_pid, qos_profile=qos.qos_profile_sensor_data # 10
          )
        
        self.create_subscription(
            Twist, '/kobe/imu/pos_angle', self.get_robot_angle, qos_profile=qos.qos_profile_sensor_data # 10
        )

        self.create_subscription(
            Twist, '/kobe/send_where_hoop', self.wherehoop, qos_profile=qos.qos_profile_sensor_data # 10
        )

        self.create_subscription(
            Twist, '/kobe/distance/shaq', self.distance, qos_profile=qos.qos_profile_sensor_data # 10
        )



        self.sent_data_timer = self.create_timer(0.01, self.sendData)
        
    def distance(self,msg):
        self.apriltag_distance = msg.linear.x
        self.distance_to_kobe = msg.linear.z
        self.middlecam_apriltag = msg.angular.x
        self.tag_id = msg.angular.z

    def wherehoop(self, msg):
        self.hoop_distance_x = msg.linear.x
        self.hoop_distance_y = msg.linear.y
        self.middlecam = msg.angular.x
        # self.middlecam = msg.angular.y


    def get_robot_angle(self,msg):
        self.yaw = WrapRads(To_Radians(msg.angular.x) * -1)

    def get_pid(self,msg):
        self.controller.ConfigPIDF(kp = msg.data[0], ki= msg.data[1], kd=msg.data[2], kf=msg.data[3]) 


    def cmd_move(self, msg):

        CurrentTime = time.time()
        self.moveSpeed = msg.linear.x
        self.turnSpeed = msg.angular.x 
        self.turnSpeed = self.turnSpeed / 3


        if self.mode == 1:
            rotation = self.controller.Calculate(WrapRads(self.yaw_setpoint - self.yaw))
            rotation = clip(rotation, -1.0, 1.0)
            if self.moveSpeed  == 0 and self.turnSpeed == 0 and abs(rotation) < 0.2:
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
            elif abs(error) < 40:  # Error threshold for small adjustments
                boost_factor = 3.0  # Increase power by 20%
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

    
        
        self.motor1Speed = (self.moveSpeed + rotation) * self.maxSpeed #Left Start Slower
        self.motor2Speed = (self.moveSpeed - rotation) * self.maxSpeed #Right
        

        if self.motor1Speed >= self.maxSpeed:
            self.motor1Speed = self.maxSpeed

        if self.motor2Speed >= self.maxSpeed:
            self.motor2Speed = self.maxSpeed    


            
        



    def cmd_shoot(self, msg):
        if not self.macro_active:  # Only update if macro is inactive
            self.motorshooter1Speed = abs(msg.linear.x - 1) * self.shootmaxSpeed
            self.motorshooter2Speed = abs(msg.linear.x - 1) * self.shootmaxSpeed
        

        self.motorshooter3Speed = abs(msg.linear.z - 1) * self.maxSpeed
        self.motorshooter3Speed += msg.angular.x * self.maxSpeed

        if self.motorshooter3Speed >= 1023.0:
            self.motorshooter3Speed = 1023.0 
            
    def cmd_macro(self, msg):

        if msg.linear.z == 1:
            self.macro_active = True
            self.motorshooter1Speed = 810.0  # Upper
            self.motorshooter2Speed = 810.0  # Lower

            # 715 --> 4300
            # 755 --> 4800

        elif msg.linear.x == 1:
            self.macro_active = True
            self.motorshooter1Speed = 660.0  # Upper
            self.motorshooter2Speed = 660.0   # Lower

            # -580 --> -1000
            # 760 --> 4800

        else:
            self.macro_active = False 


        if msg.linear.y == 1 :
            self.mode = 2

        elif msg.angular.x == 1:
            self.mode = 3
            
        else:
            self.mode = 1
         
            
    def sendData(self):
        motorspeed_msg = Twist()
        motorshooter_msg = Twist()


       
        motorspeed_msg.linear.x = float(self.motor1Speed) #Left
        motorspeed_msg.linear.y = float(self.motor2Speed) #Right


        motorshooter_msg.linear.x = float(self.motorshooter1Speed)
        motorshooter_msg.linear.y = float(self.motorshooter2Speed)
        motorshooter_msg.linear.z = float(self.motorshooter3Speed)

        self.send_shoot_speed.publish(motorshooter_msg)
        self.send_robot_speed.publish(motorspeed_msg)



def main():
    rclpy.init()

    sub = Cmd_vel_to_motor_speed()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()