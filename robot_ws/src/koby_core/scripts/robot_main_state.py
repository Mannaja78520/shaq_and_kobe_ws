#!/usr/bin/env python3
import rclpy
import math
import time

from rclpy import qos
from rclpy.node import Node
import rclpy.parameter
from std_msgs.msg import (
    Int8,
    String,
    Bool,
    Int8MultiArray,
    Float32MultiArray,
)
from geometry_msgs.msg import Twist
import numpy as np

class RobotMainState(Node):
    def __init__(self):
        super().__init__("robot_main_state_node")
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoints_start_to_challenge_zone.start', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('waypoints_start_to_challenge_zone.sub', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('waypoints_start_to_challenge_zone.challenge_zone', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('lists_challenge_zone.room1', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('lists_challenge_zone.room2', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('lists_challenge_zone.room3', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('lists_challenge_zone.room4', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('lists_challenge_zone.room5', rclpy.Parameter.Type.DOUBLE_ARRAY),
            ]
        )
        
        self.waypoint_names = [
        'start', 'sub', 'challenge_zone', 'room1', 'room2', 'room3', 'room4', 'room5'
        ]
        
        self.waypoints = {}
        for name in self.waypoint_names:
            param_name = f'waypoints_start_to_challenge_zone.{name}' if 'room' not in name else f'lists_challenge_zone.{name}'
            self.waypoints[name] = np.array(self.get_parameter(param_name).get_parameter_value().double_array_value, dtype=np.float32)
        
        self.pub_ip = self.create_publisher(
            Float32MultiArray,
            "/robot/target_nav_ip",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_goal = self.create_publisher(
            Float32MultiArray,
            "/robot/target_nav_goal",
            qos_profile=qos.qos_profile_system_default,
        )
        
        self.use_servo_pub = self.create_publisher(
            Bool,
            '/power_sub',
            qos_profile=qos.qos_profile_system_default,
        )
        
        self.pub_cancle_nav = self.create_publisher(
            Bool,
            "/robot/cancel_nav",
            qos_profile=qos.qos_profile_system_default,
        )
        self.sub_robot_state = self.create_subscription(
            String,
            "robot_state",
            self.sub_robot_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_challenge_location = self.create_subscription(
            Int8MultiArray,
            "challenge_location",
            self.sub_challenge_location_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_goal_state = self.create_subscription(
            Bool,
            "/goal/state",
            self.sub_goal_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.move_robot_pub = self.create_publisher(
            Twist, "/cmd_vel",
            qos_profile=qos.qos_profile_system_default,
        )

        self.robot_state = "None"
        self.robot_last_state = "None"
        self.__previous_goal_state = False
        self.robot_main_state: int = 0 
        self.__previous_robot_main_state: int = -1
        self.room: int = 0
        self.i: int = 0
        self.challenge_room: list = []
        self.sent_timer = self.create_timer(0.1, self.timer_callback)
        
    def sub_robot_state_callback(self, robot_state: String):
        self.robot_state = robot_state.data
        
    def sub_challenge_location_callback(self, rooms: Int8MultiArray = None) -> None:
        if self.robot_state in ["None", "Init"]:
            if rooms.data is not None:
                waypoint_map = {
                    1: self.waypoints['room1'],
                    2: self.waypoints['room2'],
                    3: self.waypoints['room3'],
                    4: self.waypoints['room4'],
                    5: self.waypoints['room5']
                }
                
                new_challenge_room = []
                
                for room in rooms.data:
                    if room in waypoint_map:
                        new_challenge_room.append(waypoint_map[room])
                # new_challenge_room.append(self.waypoints["challenge_zone"])
                
                self.challenge_room = new_challenge_room
                print(f"Updated challenge room: {self.challenge_room}")
                
    def sub_goal_state_callback(self, goal_state: Bool = False) -> None:
        if self.i < 20:
            return
        if self.__previous_goal_state != goal_state.data:
            if goal_state.data:
                self.robot_main_state += 1
            self.__previous_goal_state = goal_state.data
    
    def timer_callback(self):
        msg_ip = Float32MultiArray()
        msg_goal = Float32MultiArray()
        msg_cancle_nav = Bool(data = False)
        msg_servo_pub = Bool(data = False)
        move_robot_msg = Twist()
        print(len(self.challenge_room))
        
        if self.robot_state in ["None", "Init", "Retry"]:
            self.__previous_robot_main_state = -1
            self.i = 0
            if self.robot_state in ["None", "Retry"]:
                msg_cancle_nav.data = True
            elif self.robot_state == "Init":
                msg_ip.data = self.waypoints["start"].tolist()
                self.pub_ip.publish(msg_ip)
            time.sleep(0.25)
            self.pub_cancle_nav.publish(msg_cancle_nav)        
                
        
        if self.robot_state == "Start":
            if self.i == 0:
                self.room = 0
                self.robot_main_state = 0
                # time.sleep(0.25)
                
            if self.robot_main_state != self.__previous_robot_main_state:
                if self.robot_main_state == 0:
                    msg_goal.data = np.concatenate((self.waypoints["sub"], 
                                                    self.waypoints["challenge_zone"])).tolist()        
                    self.pub_goal.publish(msg_goal)
                elif 0 < self.robot_main_state < len(self.challenge_room)+1:
                    if self.room > 0:
                        msg_servo_pub.data = True
                        self.use_servo_pub.publish(msg_servo_pub)
                        time.sleep(6.5)
                        move_robot_msg.angular.z=-0.6
                        self.move_robot_pub.publish(move_robot_msg)
                        time.sleep(0.3)
                              
                    # msg_ip.data = (self.challenge_room[self.room - 1]).tolist()  
                    # self.pub_ip.publish(msg_ip)
                    msg_goal.data = (self.challenge_room[self.room]).tolist()        
                    self.pub_goal.publish(msg_goal)
                    self.room += 1
                elif self.robot_main_state == len(self.challenge_room)+1:
                    msg_servo_pub.data = True
                    self.use_servo_pub.publish(msg_servo_pub)
                    time.sleep(6.5)
                    move_robot_msg.angular.z=-0.6
                    self.move_robot_pub.publish(move_robot_msg)
                    time.sleep(0.3)
                    
                    # msg_ip.data = (self.challenge_room[self.room - 1]).tolist()  
                    # self.pub_ip.publish(msg_ip)
                    msg_goal.data = [float(self.waypoints["challenge_zone"][0]), float(self.waypoints["challenge_zone"][1]), -90.0]   
                    self.pub_goal.publish(msg_goal)
                    
                elif self.robot_main_state == len(self.challenge_room)+2:
                    
                    msg_goal.data = [float(self.waypoints["sub"][0] - 0.1), float(self.waypoints["sub"][1]), 0.0]   
                    self.pub_goal.publish(msg_goal)
                    
                elif self.robot_main_state == 8:
                    msg_goal.data = [float(self.waypoints["start"][0]), float(self.waypoints["start"][1]), 0.0]   
                    # self.pub_goal.publish(msg_goal)b"][1], 0.0],
                    #                                 [self.waypoints["start"][0] - 0.1, self.waypoints["
                    # msg_goal.data = np.concatenate(([self.waypoints["sub"][0] - 0.3, self.waypoints["sub"][1], 0.0],
                    #                                 [self.waypoints["start"][0] - 0.1, self.waypoints["start"][1], 0.0])).tolist()        
                    self.pub_goal.publish(msg_goal)
                    
                # elif self.robot_main_state == 7:
                #     msg_goal.data = ([self.waypoints["start"][0] - 0.11, self.waypoints["start"][1], 180]).tolist()
               
                self.__previous_robot_main_state = self.robot_main_state
            self.i += 1
        
            
def main():
    rclpy.init()
    sub = RobotMainState()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()