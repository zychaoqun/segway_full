"""--------------------------------------------------------------------
COPYRIGHT 2014 Stanley Innovation Inc.

Software License Agreement:

The software supplied herewith by Stanley Innovation Inc. (the "Company") 
for its licensed Segway RMP Robotic Platforms is intended and supplied to you, 
the Company's customer, for use solely and exclusively with Stanley Innovation 
products. The software is owned by the Company and/or its supplier, and is 
protected under applicable copyright laws.  All rights are reserved. Any use in 
violation of the foregoing restrictions may subject the user to criminal 
sanctions under applicable laws, as well as to civil liability for the 
breach of the terms and conditions of this license. The Company may 
immediately terminate this Agreement upon your use of the software with 
any products that are not Stanley Innovation products.

The software was written using Python programming language.  Your use 
of the software is therefore subject to the terms and conditions of the 
OSI- approved open source license viewable at http://www.python.org/.  
You are solely responsible for ensuring your compliance with the Python 
open source license.

You shall indemnify, defend and hold the Company harmless from any claims, 
demands, liabilities or expenses, including reasonable attorneys fees, incurred 
by the Company as a result of any claim or proceeding against the Company 
arising out of or based upon: 

(i) The combination, operation or use of the software by you with any hardware, 
    products, programs or data not supplied or approved in writing by the Company, 
    if such claim or proceeding would have been avoided but for such combination, 
    operation or use.
 
(ii) The modification of the software by or on behalf of you 

(iii) Your use of the software.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 
 \file   segway_teleop.py

 \brief  This module contains a class for teleoperating the segway
         platform with a joystick controller

 \Platform: Linux/ROS Indigo
--------------------------------------------------------------------"""
from utils import *
from system_defines import *
from segway_msgs.msg import *
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool,Float64
import rospy
import sys
import math

"""
mapping for controller order is dtz_request, powerdown_request, standby_request, tractor_request, balance_request, audio_request, 
deadman_input, manual_ovvrd_input, twist_linear_x_input, twist_linear_y_input, twist_angular_z_input
"""
MAP_DTZ_IDX         = 0
MAP_PWRDWN_IDX      = 1
MAP_STANDBY_IDX     = 2
MAP_TRACTOR_IDX     = 3
MAP_BALANCE_IDX     = 4
MAP_AUDIO_IDX       = 5
MAP_REC_GOAL_IDX    = 6
MAP_DEADMAN_IDX     = 7
MAP_MAN_OVVRD_IDX   = 8
NUMBER_OF_MOMENTARY_INPUTS = 9


MAP_TWIST_LIN_X_IDX = 0
MAP_TWIST_LIN_Y_IDX = 1 
MAP_TWIST_ANG_Z_IDX = 2
NUMBER_OF_AXIS_INPUTS = 3
 

class SegwayTeleop:
    def __init__(self):
         
        self.is_sim = rospy.get_param('~sim',False)
        
        if (False == self.is_sim):
        
            """
            Subscribe to the configuration message
            """
            self.config_updated = False
            rospy.Subscriber("/segway/feedback/configuration", Configuration, self._update_configuration_limits)
            rospy.sleep(1.0)
            
            if (False == self.config_updated):
                rospy.logerr("Timed out waiting for RMP feedback topics make sure the driver is running")
                sys.exit(0)
                return
        else:
            self.vel_limit_mps = rospy.get_param('~sim_teleop_vel_limit_mps',0.5)
            self.yaw_rate_limit_rps = rospy.get_param('~sim_teleop_yaw_rate_limit_rps',0.5)
            self.accel_lim = rospy.get_param('~sim_teleop_accel_lim',0.5)
            self.yaw_accel_lim = rospy.get_param('~sim_teleop_yaw_accel_lim',1.0)           
        
        default_ctrl_map = dict({'momentary':[[{'is_button':True,'index':4,'set_val':1}],
                                              [{'is_button':True,'index':8,'set_val':1}],
                                              [{'is_button':True,'index':1,'set_val':1}],
                                              [{'is_button':True,'index':2,'set_val':1}],
                                              [{'is_button':True,'index':0,'set_val':1}],
                                              [{'is_button':False,'index':6,'invert_axis':False,'set_thresh':0.9}],
                                              [{'is_button':False,'index':7,'invert_axis':True,'set_thresh':0.9}],
                                              [{'is_button':False,'index':2,'invert_axis':True,'set_thresh':0.9}],
                                              [{'is_button':False,'index':5,'invert_axis':True,'set_thresh':0.9}]],
                                 'axis_range':[{'index':1,'invert_axis':False},
                                               {'index':0,'invert_axis':False},
                                               {'index':3,'invert_axis':False}]})
        
        """
        Get the mapping for the various commands, defaults are xbox360 wireless
        """
        self.ctrl_map = rospy.get_param('~controller_mapping',default_ctrl_map)
        self.audio_cmd_mapped = rospy.get_param('~audio_command_to_play',MOTOR_AUDIO_TEST_SWEEP)
        
        """
        Initialize the debounce logic states
        """
        self.db_cnt = [0] * NUMBER_OF_MOMENTARY_INPUTS
        self.button_state = [False] * NUMBER_OF_MOMENTARY_INPUTS
        self.axis_value = [0.0] * NUMBER_OF_AXIS_INPUTS

        self.send_cmd_none = False
        self.no_motion_commands = True
        self.last_motion_command_time = 0.0
        self.last_joy = rospy.get_time()
            
        self.cfg_cmd = ConfigCmd()
        self.cfg_pub = rospy.Publisher('/segway/gp_command', ConfigCmd, queue_size=10)
        self.goalrecorder_pub = rospy.Publisher('/segway/record_pose',Bool, queue_size=10)
        
        self.motion_cmd = Twist()
        self.limited_cmd = Twist()
        self.motion_pub = rospy.Publisher('/segway/teleop/cmd_vel', Twist, queue_size=10)
        self.override_pub = rospy.Publisher("/segway/manual_override/cmd_vel",Twist, queue_size=10)

        rospy.sleep(1.0)
        self.cfg_cmd.header.stamp = rospy.get_rostime()
        self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_AUDIO_COMMAND'
        self.cfg_cmd.gp_param = MOTOR_AUDIO_PLAY_ENTER_ALARM_SONG
        self.cfg_pub.publish(self.cfg_cmd)

        rospy.Subscriber('/joy', Joy, self._segway_teleop)
        
    def _update_configuration_limits(self,config):
        
        self.vel_limit_mps = config.teleop_vel_limit_mps
        self.yaw_rate_limit_rps = config.teleop_yaw_rate_limit_rps
        self.accel_lim = config.teleop_accel_limit_mps2
        self.yaw_accel_lim = config.teleop_yaw_accel_limit_rps2
        self.config_updated = True
        
    def _parse_joy_input(self,joyMessage):
        

        raw_button_states = [True] * NUMBER_OF_MOMENTARY_INPUTS
        self.button_state = [False] * NUMBER_OF_MOMENTARY_INPUTS
         
        for i in range(NUMBER_OF_MOMENTARY_INPUTS):
            inputs_for_req = self.ctrl_map['momentary'][i]
            for item in inputs_for_req:
                if item['is_button']:
                    if item['set_val'] == joyMessage.buttons[item['index']]:
                        raw_button_states[i] &= True
                    else:
                        raw_button_states[i] = False
                else:
                    temp = joyMessage.axes[item['index']]
                    if (item['invert_axis']):
                        temp *= -1.0
                    if (temp >= item['set_thresh']):
                        raw_button_states[i] &= True
                    else:
                        raw_button_states[i] = False
                     
            
            if (True == raw_button_states[i]):
                self.db_cnt[i]+=1
                if (self.db_cnt[i] > 10):
                    self.db_cnt[i] = 10
                    self.button_state[i] = True
            else:
                self.button_state[i] = False
                self.db_cnt[i] = 0
        
        self.axis_value = [0.0] * NUMBER_OF_AXIS_INPUTS
        for i in range(NUMBER_OF_AXIS_INPUTS):
            axis_input_map = self.ctrl_map['axis_range'][i]
            temp = joyMessage.axes[axis_input_map['index']]
            if (axis_input_map['invert_axis']):
                temp *= -1.0
            self.axis_value[i] = temp

    def _segway_teleop(self, joyMessage):
        self._parse_joy_input(joyMessage)
        if self.button_state[MAP_REC_GOAL_IDX] == 1:
            if (False == self.goalrecorded):
                temp = Bool()
                temp.data = True
                self.goalrecorder_pub.publish(temp)
                self.goalrecorded= True
        else:
            self.goalrecorded= False                                  

        if self.button_state[MAP_DTZ_IDX]:
            self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            self.cfg_cmd.gp_param = DTZ_REQUEST
        elif self.button_state[MAP_PWRDWN_IDX]:
            self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            self.cfg_cmd.gp_param = STANDBY_REQUEST
        elif self.button_state[MAP_STANDBY_IDX]:
            self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            self.cfg_cmd.gp_param = STANDBY_REQUEST
        elif self.button_state[MAP_TRACTOR_IDX]:
            self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            self.cfg_cmd.gp_param = TRACTOR_REQUEST
        elif self.button_state[MAP_BALANCE_IDX]:
            self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
            self.cfg_cmd.gp_param = BALANCE_REQUEST
        elif self.button_state[MAP_AUDIO_IDX]:
            self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_AUDIO_COMMAND'
            self.cfg_cmd.gp_param = self.audio_cmd_mapped
        else:
            self.cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
            self.cfg_cmd.gp_param = 0
            
        if ('GENERAL_PURPOSE_CMD_NONE' != self.cfg_cmd.gp_cmd):
            self.cfg_cmd.header.stamp = rospy.get_rostime()
            self.cfg_pub.publish(self.cfg_cmd)
            self.cfg_cmd.header.seq
            self.send_cmd_none = True
        elif (True == self.send_cmd_none):
            self.cfg_cmd.header.stamp = rospy.get_rostime()
            self.cfg_pub.publish(self.cfg_cmd)
            self.cfg_cmd.header.seq
            self.send_cmd_none = False
        elif (False == self.send_cmd_none):
            if self.button_state[MAP_DEADMAN_IDX]:
                self.motion_cmd.linear.x =  (self.axis_value[MAP_TWIST_LIN_X_IDX] * self.vel_limit_mps)
                self.motion_cmd.linear.y =  (self.axis_value[MAP_TWIST_LIN_Y_IDX] * self.vel_limit_mps)
                self.motion_cmd.angular.z = (self.axis_value[MAP_TWIST_ANG_Z_IDX] * self.yaw_rate_limit_rps)
                self.last_motion_command_time = rospy.get_time()
              
            else:
                self.motion_cmd.linear.x = 0.0
                self.motion_cmd.linear.y = 0.0
                self.motion_cmd.angular.z = 0.0

            dt = rospy.get_time() - self.last_joy
            self.last_joy = rospy.get_time()
            
            if (dt >= 0.01):

                self.limited_cmd.linear.x = slew_limit(self.motion_cmd.linear.x,
                                                       self.limited_cmd.linear.x,
                                                       self.accel_lim, dt)
                self.limited_cmd.linear.y = slew_limit(self.motion_cmd.linear.y,
                                                       self.limited_cmd.linear.y,
                                                       self.accel_lim, dt)
                self.limited_cmd.angular.z = slew_limit(self.motion_cmd.angular.z,
                                                       self.limited_cmd.angular.z,
                                                       self.yaw_accel_lim, dt)

                if ((rospy.get_time() - self.last_motion_command_time) < 2.0):
     
                    
                    self.motion_pub.publish(self.limited_cmd)
                    
                    if self.button_state[MAP_DEADMAN_IDX] and self.button_state[MAP_MAN_OVVRD_IDX]:
                        self.override_pub.publish(self.motion_cmd)

           
        
        


    
