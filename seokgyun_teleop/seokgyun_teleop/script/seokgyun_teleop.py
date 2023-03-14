#! /usr/bin/env python
import rclpy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import os, sys, select
from piot_can_msgs.msg import CtrlCmd, CtrlFb ,SteeringCtrlCmd,FrontAngleFreeCtrlCmd,RearAngleFreeCtrlCmd,FrontVelocityFreeCtrlCmd,RearVelocityFreeCtrlCmd


if os.name == 'nt':
	import msvcrt
else:
	import termios
	import tty

PIOT_MAX_LIN_VEL = 2 # m/s
PIOT_MAX_ANG_VEL = 3.14 # rad/s


LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.1


msg = """
Moving around:
        w
   a    s    d
        x

normal mode:
    w/x : increase/decrease linear velocity 
    a/d : increase/decrease angular velocity

parallel mode:
    w/x : increase/decrease linear x  velocity 
    a/d : increase/decrease linear y  velocity

s : force stop
space key : mode change

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
	if os.name == 'nt':
		return msvcrt.getch().decode('utf-8')
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def print_vels(self):
	print('currently:\n linear vel x {0}\n linear vel y {1}\n angular vel {2}\n steering_cmd_vel {3}\n steering_cmd_steering {4}\n steering_cmd_slipangle {5}\n free_angle_lf {6}\n free_angle_rf {7}\n free_angle_lr {8}\n free angle_rr {9}\n free_vel_lf {10}\n free_vel_rf {11}\n free_vel_lr {12}\n free_vel_rr {13}'
	.format(self.target_linear_x_vel,self.target_linear_y_vel,self.target_angular_vel,self.target_steering_cmd_velocity,self.target_steering_cmd_steering,self.target_steering_cmd_slipangle,self.target_free_cmd_angle_lf,self.target_free_cmd_angle_rf,self.target_free_cmd_angle_lr,self.target_free_cmd_angle_rr,self.target_free_cmd_velocity_lf,self.target_free_cmd_velocity_rf,self.target_free_cmd_velocity_lr,self.target_free_cmd_velocity_rr))


def make_simple_profile(output, input, slop):
	if input > output:
		output = min( input, output + slop )
	elif input < output:
		output = max( input, output - slop )
	else:
		output = input

	return output


def constrain(input, low, high):
	if input < low:
		input = low
	elif input > high:
		input = high
	else:
		input = input
	return input


def check_linear_limit_velocity(vel):
	return constrain(vel, -PIOT_MAX_LIN_VEL, PIOT_MAX_LIN_VEL)
	

def check_angular_limit_velocity(vel):
	return constrain(vel, -PIOT_MAX_ANG_VEL, PIOT_MAX_ANG_VEL)


class robot:
	def __init__(self):
		self.mode = Bool()
		self.mode_flag= 'normal'
		self.con_idx = 0
		self.control_mode = ['ctrl_cmd','steering_cmd','front_ang_cmd','rear_ang_cmd','front_vel_cmd','rear_vel_cmd']
		self.target_ctrl_cmd_gear = 1 # 
		###############################
		self.target_linear_x_vel   = 0.0
		self.target_linear_y_vel   = 0.0
		self.target_angular_vel  = 0.0
		self.control_linear_x_vel = 0.0
		self.control_linear_y_vel = 0.0
		self.control_angular_vel = 0.0

		self.target_steering_cmd_velocity = 0.0
		self.target_steering_cmd_steering = 0.0
		self.target_steering_cmd_slipangle = 0.0
		self.control_steering_cmd_velocity = 0.0
		self.control_steering_cmd_steering = 0.0
		self.control_steering_cmd_slipangle = 0.0

		self.target_free_cmd_angle_lf = 0.0
		self.target_free_cmd_angle_rf = 0.0
		self.control_free_cmd_angle_lf = 0.0
		self.control_free_cmd_angle_rf = 0.0

		self.target_free_cmd_angle_rr = 0.0
		self.target_free_cmd_angle_lr = 0.0
		self.control_free_cmd_angle_rr = 0.0
		self.control_free_cmd_angle_lr = 0.0

		self.target_free_cmd_velocity_lf = 0.0
		self.target_free_cmd_velocity_rf = 0.0
		self.control_free_cmd_velocity_lf = 0.0
		self.control_free_cmd_velocity_rf = 0.0

		self.target_free_cmd_velocity_lr = 0.0
		self.target_free_cmd_velocity_rr = 0.0
		self.control_free_cmd_velocity_lr = 0.0
		self.control_free_cmd_velocity_rr = 0.0


	def e_stop(self):
		self.target_linear_x_vel   = 0.0
		self.target_linear_y_vel   = 0.0
		self.target_angular_vel  = 0.0


		self.target_steering_cmd_velocity = 0.0
		self.target_steering_cmd_steering = 0.0
		self.target_steering_cmd_slipangle = 0.0

		self.target_free_cmd_angle_lf = 0.0
		self.target_free_cmd_angle_rf = 0.0

		self.target_free_cmd_angle_rr = 0.0
		self.target_free_cmd_angle_lr = 0.0

		self.target_free_cmd_velocity_lf = 0.0
		self.target_free_cmd_velocity_rf = 0.0

		self.target_free_cmd_velocity_lr = 0.0

def main():
	settings = None
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)

	rclpy.init()
	
	qos = QoSProfile(depth=10)
	node = rclpy.create_node('seokgyun_teleop_node')
	robot_ = robot()
	cmd_vel_pub = node.create_publisher( Twist, 'cmd_vel', qos)
	cmd_steering_pub = node.create_publisher (SteeringCtrlCmd, 'steering_ctrl_cmd', qos)
	mode_pub = node.create_publisher( Bool, 'mode', qos)

	try:
		print(msg)
		while(1):
			key = get_key(settings)
			if key == 'w' :
				if robot_.control_mode[robot_.con_idx] == 'ctrl_cmd':
					robot_.target_linear_x_vel = check_linear_limit_velocity(robot_.target_linear_x_vel + LIN_VEL_STEP_SIZE)
				elif robot_.control_mode[robot_.con_idx] == 'steering_cmd':
					robot_.target_steering_cmd_velocity = check_linear_limit_velocity(robot_.target_steering_cmd_velocity +LIN_VEL_STEP_SIZE)
				print_vels(robot_)

			elif key == 'x' :
				if robot_.control_mode[robot_.con_idx] == 'ctrl_cmd':
					robot_.target_linear_x_vel = check_linear_limit_velocity(robot_.target_linear_x_vel - LIN_VEL_STEP_SIZE)
				elif robot_.control_mode[robot_.con_idx] == 'steering_cmd':
					robot_.target_steering_cmd_velocity = check_linear_limit_velocity(robot_.target_steering_cmd_velocity - LIN_VEL_STEP_SIZE)
				print_vels(robot_)

			elif key == 'a' :
				if robot_.mode_flag == 'normal':
					if robot_.control_mode[robot_.con_idx] =='steering_cmd':
						robot_.target_steering_cmd_steering = check_angular_limit_velocity(robot_.target_steering_cmd_steering + ANG_VEL_STEP_SIZE)
					else :	
						robot_.target_angular_vel = check_angular_limit_velocity(robot_.target_angular_vel + ANG_VEL_STEP_SIZE)
				else :
					robot_.target_linear_y_vel = check_linear_limit_velocity(robot_.target_linear_y_vel + LIN_VEL_STEP_SIZE)
				print_vels(robot_)

			elif key == 'd' :
				if robot_.mode_flag == 'normal':
					if robot_.control_mode[robot_.con_idx] =='steering_cmd':
						robot_.target_steering_cmd_steering = check_angular_limit_velocity(robot_.target_steering_cmd_steering - ANG_VEL_STEP_SIZE)
					else :	
						robot_.target_angular_vel = check_angular_limit_velocity(robot_.target_angular_vel - ANG_VEL_STEP_SIZE)
				else :
					robot_.target_linear_y_vel = check_linear_limit_velocity(robot_.target_linear_y_vel - LIN_VEL_STEP_SIZE)
				print_vels(robot_)

			elif key == 's' :
				robot_.e_stop()
				print_vels(robot_)
			elif key == ' ' :
				if robot_.mode_flag =='normal':
					robot_.mode_flag ='parallel'
				else :
					robot_.mode_flag ='normal'
				robot_.target_linear_x_vel   = 0.0
				robot_.control_linear_x_vel  = 0.0
				robot_.target_linear_y_vel   = 0.0
				robot_.control_linear_y_vel  = 0.0
				robot_.target_angular_vel  = 0.0
				robot_.control_angular_vel = 0.0
				print("Mode change : {}".format(robot_.mode_flag))
			elif key == '/':
				if robot_.con_idx == 5:
					robot_.con_idx = 0
				else :
					robot_.con_idx = robot_.con_idx+1 
				print('현재 모드는 {0}'.format(robot_.control_mode[robot_.con_idx]))
			

					
			twist = Twist()
			steering_cmd = SteeringCtrlCmd()
			mode = Bool()
			if robot_.control_mode[robot_.con_idx]=='ctrl_cmd':
				
				if robot_.mode_flag =='normal':
					mode.data = True
				else :
					mode.data = False
				twist.linear.x = robot_.target_linear_x_vel 
				twist.linear.y = robot_.target_linear_y_vel
				twist.angular.z = robot_.target_angular_vel  
				cmd_vel_pub.publish(twist)
				mode_pub.publish(mode)
			elif robot_.control_mode[robot_.con_idx] =='steering_cmd':
				
				if robot_.mode_flag == 'normal':
					steering_cmd.ctrl_cmd_gear = 6
				if robot_.mode_flag == 'parallel':
					steering_cmd.ctrl_cmd_gear = 7
				steering_cmd.steering_ctrl_cmd_velocity= robot_.target_steering_cmd_velocity 
				steering_cmd.steering_ctrl_cmd_steering= robot_.target_steering_cmd_steering * 0.57
				steering_cmd._steering_ctrl_cmd_slipangle= robot_.target_steering_cmd_slipangle * 0.57
				cmd_steering_pub.publish(steering_cmd)


	except Exception as e:
		print(e)

	finally:
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
		cmd_vel_pub.publish(twist)
		
		if os.name !='nt':
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
			
if __name__ == '__main__':
	main()
