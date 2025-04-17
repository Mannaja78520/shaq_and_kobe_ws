#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray , Float32MultiArray
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy
from rclpy import qos

class Gamepad:
    def __init__(self):
        #Axes:--------------------------------------------------------
        
        self.lx : float = 0.0                   # 0: Left X-Axis
        self.ly : float = 0.0                   # 1: Left Y-Axis
        self.l2 : float = 0.0                   # 2: L2
        self.rx : float = 0.0                   # 3: Right X-Axis
        self.ry : float = 0.0                   # 4: Right Y-Axis
        self.r2 : float = 0.0                   # 5: R2
        self.dpadLeftRight : float = 0.0        # 6: Dpad Left and Right
        self.dpadUpDown : float = 0.0           # 7: Dpad Up and Down
        
        #Buttons:-------------------------------------------------------
        
        self.button_cross : float = 0.0         # 0: 
        self.button_circle : float = 0.0        # 1:
        self.button_triangle : float = 0.0      # 2:
        self.button_square : float = 0.0        # 3:
        self.l1 : float = 0.0                   # 4:
        self.r1 : float = 0.0                   # 5:
        #self.l2_logic : float = 0.0                   # 6:
        #self.r2_logic : float = 0.0                   # 7:
        self.button_share : float = 0.0         # 8:
        self.button_option : float = 0.0        # 9:
        self.button_logo : float = 0.0          # 10:
        self.PressedLeftAnalog : float = 0.0    # 11:
        self.PressedRightAnalog : float = 0.0   # 12:

        #----------------------------------------------------------------
        
        self.auto_aim_bool: bool =False
        self.dribble: bool = False 
        self.toggle_shoot_bool : bool = False 
        self.toggle_pass_bool : bool = False
        self.toggle_pass_motor_bool : bool = False

        self.previous_triangle_state = False
        self.previous_circle_state = False
        self.previous_cross_state = False
        self.previous_square_state = False
        self.previous_l1_state = False

        self.last_macro_button = None  # Stores 'shoot', 'pass', 'dribble', 'auto_aim' , 'pass_motor'

        
        
    # def update_dribble(self):

    #     if self.button_triangle and not self.previous_triangle_state:
    
    #         self.dribble = not self.dribble  # Toggle state
            
    #     self.previous_triangle_state = self.button_triangle  # Update button state

    # def update_auto_aim(self):

    #     if self.button_circle and not self.previous_circle_state:
    
    #         self.auto_aim_bool = not self.auto_aim_bool  # Toggle state
            
    #     self.previous_circle_state = self.button_circle  # Update button state

    # def update_toggle_shoot(self):

    #     if self.button_cross and not self.previous_cross_state:

    #         self.toggle_shoot_bool = not self.toggle_shoot_bool  # Toggle state

    #     self.previous_cross_state = self.button_cross  # Update button state

    # def update_toggle_pass(self):

    #     if self.button_square and not self.previous_square_state:

    #         self.toggle_pass_bool = not self.toggle_pass_bool  # Toggle state

    #     self.previous_square_state = self.button_square  # Update button state

    def update_dribble(self):
        if self.button_triangle and not self.previous_triangle_state:
            self.dribble = not self.dribble
            if self.dribble:
                # Disable others
                self.auto_aim_bool = False
                self.toggle_shoot_bool = False
                self.toggle_pass_bool = False
                self.toggle_pass_motor_bool = False
                self.last_macro_button = 'dribble'
            else:
                self.last_macro_button = None
        self.previous_triangle_state = self.button_triangle

    def update_auto_aim(self):
        if self.button_circle and not self.previous_circle_state:
            self.auto_aim_bool = not self.auto_aim_bool
            if self.auto_aim_bool:
                self.dribble = False
                self.toggle_shoot_bool = False
                self.toggle_pass_bool = False
                self.toggle_pass_motor_bool = False
                self.last_macro_button = 'auto_aim'
            else:
                self.last_macro_button = None
        self.previous_circle_state = self.button_circle

    def update_toggle_shoot(self):
        if self.button_cross and not self.previous_cross_state:
            self.toggle_shoot_bool = not self.toggle_shoot_bool
            if self.toggle_shoot_bool:
                self.dribble = False
                self.auto_aim_bool = False
                self.toggle_pass_bool = False
                self.toggle_pass_motor_bool = False
                self.last_macro_button = 'shoot'
            else:
                self.last_macro_button = None
        self.previous_cross_state = self.button_cross

    def update_toggle_pass(self):
        if self.button_square and not self.previous_square_state:
            self.toggle_pass_bool = not self.toggle_pass_bool
            if self.toggle_pass_bool:
                self.toggle_shoot_bool = False
                self.dribble = False
                self.auto_aim_bool = False
                self.toggle_pass_motor_bool = False
                self.last_macro_button = 'pass'
            else:
                self.last_macro_button = None
        self.previous_square_state = self.button_square

    def update_toggle_pass_motor(self):
        if self.l1 and not self.previous_l1_state:
            self.toggle_pass_motor_bool = not self.toggle_pass_motor_bool
            if self.toggle_pass_motor_bool:
                self.toggle_shoot_bool = False
                self.dribble = False
                self.auto_aim_bool = False
                self.toggle_pass_bool = False
                self.last_macro_button = 'pass_motor'
            else:
                self.last_macro_button = None
        self.previous_l1_state = self.l1

    def reset_toggles(self):
        self.auto_aim_bool = False
        self.dribble = False 
        self.toggle_shoot_bool = False 
        self.toggle_pass_bool = False
        self.toggle_pass_motor_bool = False
        self.last_macro_button = None
        # add any others here

        


class Joystick(Node):
    def __init__(self):
        super().__init__("joystick")

        self.pub_move = self.create_publisher(
            Twist, "/kobe/cmd_move", qos_profile=qos.qos_profile_system_default
        )
        
        self.pub_macro = self.create_publisher(
            Twist, "/kobe/cmd_macro", qos_profile=qos.qos_profile_system_default
        )

        self.pub_shoot = self.create_publisher(
            Twist, "/kobe/cmd_shoot", qos_profile=qos.qos_profile_system_default
        )
        
        self.create_subscription(
            Joy, '/kobe/joy', self.joy, qos_profile=qos.qos_profile_sensor_data # 10
        )

        self.gamepad = Gamepad()
        self.maxspeed : float = 1.0
        


        self.sent_data_timer = self.create_timer(0.01, self.sendData)

    def joy(self, msg):
        
        #Axes:--------------------------------------------------------
        
        self.gamepad.lx = float(msg.axes[0] * -1)                   # 0: Left X-Axis
        self.gamepad.ly = float(msg.axes[1])                        # 1: Left Y-Axis
        self.gamepad.l2 = float((msg.axes[2] + 1)/ 2)               # 2: L2
        self.gamepad.rx = float(msg.axes[3] * -1)                   # 3: Right X-Axis
        self.gamepad.ry = float(msg.axes[4])                        # 4: Right Y-Axis
        self.gamepad.r2 = float((msg.axes[5] + 1)/ 2)               # 5: R2
        self.gamepad.dpadLeftRight  = float(msg.axes[6])            # 6: Dpad Left and Right
        self.gamepad.dpadUpDown     = float(msg.axes[7])            # 7: Dpad Up and Down
        
        #Buttons:-------------------------------------------------------

        self.gamepad.button_cross    = float(msg.buttons[0])        # 0: 
        self.gamepad.button_circle   = float(msg.buttons[1])        # 1:
        self.gamepad.button_triangle = float(msg.buttons[2])        # 2:
        self.gamepad.button_square   = float(msg.buttons[3])        # 3:
        self.gamepad.l1              = float(msg.buttons[4])        # 4:
        self.gamepad.r1              = float(msg.buttons[5])        # 5:
        #self.gamepad.l2              = float(msg.buttons[6])        # 6:
        #self.gamepad.r2              = float(msg.buttons[7])        # 7:
        self.gamepad.button_share    = float(msg.buttons[8])        # 8:
        self.gamepad.button_option   = float(msg.buttons[9])        # 9:
        self.gamepad.button_logo     = float(msg.buttons[10])       # 10:
        self.gamepad.PressedLeftAnalog  = float(msg.buttons[11])    # 11:
        self.gamepad.PressedRightAnalog = float(msg.buttons[12])    # 12:
        
        
        #Macro-----------------------------------------------------------
        
        self.gamepad.update_dribble()
        self.gamepad.update_auto_aim()
        self.gamepad.update_toggle_shoot()
        self.gamepad.update_toggle_pass()
        self.gamepad.update_toggle_pass_motor()
    
        
        if self.gamepad.button_logo:
            self.gamepad.reset_toggles()


        
        

    def sendData(self):
        
        cmd_vel_move = Twist()
        cmd_vel_shoot = Twist()
        cmd_vel_macro = Twist()


        cmd_vel_move.linear.x = float(self.gamepad.ly * self.maxspeed)
        cmd_vel_move.angular.x = float(self.gamepad.rx * self.maxspeed)
        
        cmd_vel_shoot.linear.x = float(self.gamepad.r2 * self.maxspeed)
        cmd_vel_shoot.linear.y = float(self.gamepad.r2 * self.maxspeed)
        cmd_vel_shoot.linear.z = float(self.gamepad.l2 * self.maxspeed)
        cmd_vel_shoot.angular.x = float(self.gamepad.dpadUpDown * self.maxspeed)

        # if self.gamepad.toggle_shoot_bool:
        #     cmd_vel_macro.linear.z = 1.0

        # else:
        #     cmd_vel_macro.linear.z = 0.0

        
        # if self.gamepad.auto_aim_bool:
        #     cmd_vel_macro.linear.y = 1.0
            
        # else:
        #     cmd_vel_macro.linear.y = 0.0
        
        
        # if self.gamepad.dribble:
        #     cmd_vel_macro.linear.x = 1.0
              
        # else:
        #     cmd_vel_macro.linear.x = 0.0  

        # if self.gamepad.toggle_pass_bool:
        #     cmd_vel_macro.angular.x = 1.0

        # else:
        #     cmd_vel_macro.angular.x = 0.0    


            

        if self.gamepad.last_macro_button == 'dribble' and self.gamepad.dribble:
            cmd_vel_macro.linear.x = 1.0

        elif self.gamepad.last_macro_button == 'auto_aim' and self.gamepad.auto_aim_bool:
            cmd_vel_macro.linear.y = 1.0

        elif self.gamepad.last_macro_button == 'shoot' and self.gamepad.toggle_shoot_bool:
            cmd_vel_macro.linear.z = 1.0

        elif self.gamepad.last_macro_button == 'pass' and self.gamepad.toggle_pass_bool:
            cmd_vel_macro.angular.x = 1.0

        elif self.gamepad.last_macro_button == 'pass_motor' and self.gamepad.toggle_pass_motor_bool:
            cmd_vel_macro.angular.y = 1.0
        
    
        self.pub_macro.publish(cmd_vel_macro)
        self.pub_move.publish(cmd_vel_move)
        self.pub_shoot.publish(cmd_vel_shoot)



def main():
    rclpy.init()

    sub = Joystick()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
