from utils import *
from system_defines import *
from movo_msgs.msg import *
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
MAP_IS_ROT_IDX = 0
NUMBER_OF_MOMENTARY_INPUTS = 1

# Note that Twist Y is Unity X and vice versa
MAP_TWIST_LIN_X_IDX = 1
MAP_TWIST_LIN_Y_IDX = 0
NUMBER_OF_AXIS_INPUTS = 2
IS_X_INVERSE = False
IS_Y_INVERSE = True
 

class MovoTeleopVR:
    def __init__(self):
        self.is_sim = rospy.get_param('~sim',False)
        
        if (False == self.is_sim):
        
            """
            Subscribe to the configuration message
            """
            self.config_updated = False
            rospy.Subscriber("/movo/feedback/active_configuration", Configuration, self._update_configuration_limits)
            
            start_time = rospy.get_time()            
            while ((rospy.get_time() - start_time) < 10.0) and (False == self.config_updated):
                rospy.sleep(0.05)
            
            if (False == self.config_updated):
                rospy.logerr("Timed out waiting for Movo feedback topics make sure the driver is running")
                sys.exit(0)
                return
        else:
            self.x_vel_limit_mps = rospy.get_param('~sim_teleop_x_vel_limit_mps',0.5)
            self.y_vel_limit_mps = rospy.get_param('~sim_teleop_y_vel_limit_mps',0.5)
            self.yaw_rate_limit_rps = rospy.get_param('~sim_teleop_yaw_rate_limit_rps',0.5)
            self.accel_lim = rospy.get_param('~sim_teleop_accel_lim',0.5)
            self.yaw_accel_lim = rospy.get_param('~sim_teleop_yaw_accel_lim',1.0)           
        
        """
        Initialize the debounce logic states
        """
        self.button_state = [False] * NUMBER_OF_MOMENTARY_INPUTS
        self.axis_value = [0.0] * NUMBER_OF_AXIS_INPUTS

        self.send_cmd_none = False
        self.no_motion_commands = True
        self.last_motion_command_time = 0.0
        self.last_joy = rospy.get_time()
            
        self.cfg_cmd = ConfigCmd()
        self.cfg_pub = rospy.Publisher('/movo/gp_command', ConfigCmd, queue_size=10)
        self.goalrecorder_pub = rospy.Publisher('/movo/record_pose',Bool, queue_size=10)
        
        self.motion_cmd = Twist()
        self.limited_cmd = Twist()
        self.motion_pub = rospy.Publisher('/movo/teleop/cmd_vel', Twist, queue_size=10)
        self.override_pub = rospy.Publisher("/movo/manual_override/cmd_vel",Twist, queue_size=10)

        rospy.Subscriber('/joy', Joy, self._movo_teleop)
        
    def _update_configuration_limits(self,config):
        
        self.x_vel_limit_mps = config.teleop_x_vel_limit_mps
        self.y_vel_limit_mps = config.teleop_y_vel_limit_mps
        self.yaw_rate_limit_rps = config.teleop_yaw_rate_limit_rps
        self.accel_lim = config.teleop_accel_limit_mps2
        self.yaw_accel_lim = config.teleop_yaw_accel_limit_rps2
        self.config_updated = True
        
    def _parse_joy_input(self,joyMessage):

        self.button_state = [False] * NUMBER_OF_MOMENTARY_INPUTS
         
        self.button_state[0] = joyMessage.buttons[MAP_IS_ROT_IDX] == 1

        self.axis_value = [0.0] * NUMBER_OF_AXIS_INPUTS
        self.axis_value[MAP_TWIST_LIN_X_IDX] = joyMessage.axes[MAP_TWIST_LIN_X_IDX]
        self.axis_value[MAP_TWIST_LIN_Y_IDX] = joyMessage.axes[MAP_TWIST_LIN_Y_IDX]

        if IS_X_INVERSE:
            self.axis_value[MAP_TWIST_LIN_X_IDX] *= -1.0
        if IS_Y_INVERSE:
            self.axis_value[MAP_TWIST_LIN_Y_IDX] *= -1.0


    def _movo_teleop(self, joyMessage):
        self._parse_joy_input(joyMessage)
        
        if self.button_state[0]:
            self.motion_cmd.linear.x =  0.0
            self.motion_cmd.linear.y =  0.0
            self.motion_cmd.angular.z = self.axis_value[MAP_TWIST_LIN_Y_IDX]
        else:
            self.motion_cmd.linear.x =  (self.axis_value[MAP_TWIST_LIN_X_IDX] * self.x_vel_limit_mps)
            self.motion_cmd.linear.y =  (self.axis_value[MAP_TWIST_LIN_Y_IDX] * self.y_vel_limit_mps)
            self.motion_cmd.angular.z = 0.0
        self.last_motion_command_time = rospy.get_time()


        dt = rospy.get_time() - self.last_joy
        self.last_joy = rospy.get_time()
        
        if (dt >= 0.01):
            if ((rospy.get_time() - self.last_motion_command_time) < 2.0):  
                self.motion_pub.publish(self.motion_cmd)

           
        
        


    
