#! /usr/bin/env python
import rclpy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import os, sys, select
from piot_can_msgs.msg import CtrlCmd, CtrlFb ,SteeringCtrlCmd,FrontAngleFreeCtrlCmd,RearAngleFreeCtrlCmd,FrontVelocityFreeCtrlCmd,RearVelocityFreeCtrlCmd ,IoCmd
import math

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
	print('currently:\n linear vel x {0}\n linear vel y {1}\n angular vel {2}\n steering_cmd_vel {3}\n steering_cmd_steering {4}\n steering_cmd_slipangle {5}\n free_angle_lf {6}\n free_angle_rf {7}\n free_angle_lr {8}\n free angle_rr {9}\n free_vel_lf {10}\n free_vel_rf {11}\n free_vel_lr {12}\n free_vel_rr {13}\n io_cmd_unlock {14}\n io_cmd_speaker {15}'
	.format(self.target_linear_x_vel,self.target_linear_y_vel,self.target_angular_vel,self.target_steering_cmd_velocity,self.target_steering_cmd_steering,self.target_steering_cmd_slipangle,self.target_free_cmd_angle_lf,self.target_free_cmd_angle_rf,self.target_free_cmd_angle_lr,self.target_free_cmd_angle_rr,self.target_free_cmd_velocity_lf,self.target_free_cmd_velocity_rf,self.target_free_cmd_velocity_lr,self.target_free_cmd_velocity_rr,self.io_cmd_unlock,self.io_cmd_speaker))


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
		self.io_cmd_unlock = False
		self.io_cmd_speaker = False
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
		self.gear =1

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
	# cmd_vel_pub = node.create_publisher( Twist, 'cmd_vel', qos)
	ctrl_cmd_pub = node.create_publisher (CtrlCmd, 'ctrl_cmd', qos)
	cmd_steering_pub = node.create_publisher (SteeringCtrlCmd, 'steering_ctrl_cmd', qos)
	faf_cmd_pub = node.create_publisher (FrontAngleFreeCtrlCmd, 'front_angle_free_ctrl_cmd', qos)
	raf_cmd_pub = node.create_publisher (RearAngleFreeCtrlCmd, 'rear_angle_free_ctrl_cmd', qos)
	fvf_cmd_pub = node.create_publisher (FrontVelocityFreeCtrlCmd, 'front_velocity_free_ctrl_cmd', qos)
	rvf_cmd_pub = node.create_publisher (RearVelocityFreeCtrlCmd, 'rear_velocity_free_ctrl_cmd', qos)
	# mode_pub = node.create_publisher( Bool, 'mode', qos)
	io_cmd_pub = node.create_publisher (IoCmd, 'io_cmd', qos)
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
				if robot_.gear == 6:
					if robot_.control_mode[robot_.con_idx] =='steering_cmd':
						robot_.target_steering_cmd_steering = check_angular_limit_velocity(robot_.target_steering_cmd_steering + ANG_VEL_STEP_SIZE)
					else :	
						robot_.target_angular_vel = check_angular_limit_velocity(robot_.target_angular_vel + ANG_VEL_STEP_SIZE)
				else :
					robot_.target_linear_y_vel = check_linear_limit_velocity(robot_.target_linear_y_vel + LIN_VEL_STEP_SIZE)
				print_vels(robot_)

			elif key == 'd' :
				if robot_.gear ==  6 :
					if robot_.control_mode[robot_.con_idx] =='steering_cmd':
						robot_.target_steering_cmd_steering = check_angular_limit_velocity(robot_.target_steering_cmd_steering - ANG_VEL_STEP_SIZE)
					else :	
						robot_.target_angular_vel = check_angular_limit_velocity(robot_.target_angular_vel - ANG_VEL_STEP_SIZE)
				else :
					robot_.target_linear_y_vel = check_linear_limit_velocity(robot_.target_linear_y_vel - LIN_VEL_STEP_SIZE)
				print_vels(robot_)

			elif key == 'q' :
				if robot_.control_mode[robot_.con_idx] == 'front_ang_cmd':
					robot_.target_free_cmd_angle_lf = check_angular_limit_velocity(robot_.target_free_cmd_angle_lf  + ANG_VEL_STEP_SIZE)

				elif robot_.control_mode[robot_.con_idx] == 'front_vel_cmd':
					robot_.target_free_cmd_velocity_lf = check_linear_limit_velocity(robot_.target_free_cmd_velocity_lf  + LIN_VEL_STEP_SIZE)

				print_vels(robot_)

			elif key == 'e' :
				if robot_.control_mode[robot_.con_idx] == 'front_ang_cmd':
					robot_.target_free_cmd_angle_rf = check_angular_limit_velocity(robot_.target_free_cmd_angle_rf  + ANG_VEL_STEP_SIZE)	

				elif robot_.control_mode[robot_.con_idx] == 'front_vel_cmd':
					robot_.target_free_cmd_velocity_rf = check_linear_limit_velocity(robot_.target_free_cmd_velocity_rf  + LIN_VEL_STEP_SIZE)

				print_vels(robot_)

			elif key == 'z' :

				if robot_.control_mode[robot_.con_idx] == 'rear_ang_cmd':
					robot_.target_free_cmd_angle_lr = check_angular_limit_velocity(robot_.target_free_cmd_angle_lr  + ANG_VEL_STEP_SIZE)

				elif robot_.control_mode[robot_.con_idx] == 'rear_vel_cmd':
					robot_.target_free_cmd_velocity_lr = check_linear_limit_velocity(robot_.target_free_cmd_velocity_lr  + LIN_VEL_STEP_SIZE)

				print_vels(robot_)

			elif key == 'c' :

				if robot_.control_mode[robot_.con_idx] == 'rear_ang_cmd':
					robot_.target_free_cmd_angle_rr = check_angular_limit_velocity(robot_.target_free_cmd_angle_rr  + ANG_VEL_STEP_SIZE)
				elif robot_.control_mode[robot_.con_idx] == 'rear_vel_cmd':
					robot_.target_free_cmd_velocity_rr = check_linear_limit_velocity(robot_.target_free_cmd_velocity_rr  + LIN_VEL_STEP_SIZE)

				print_vels(robot_)
			elif key == 'p':
				if robot_.io_cmd_unlock == False :
					robot_.io_cmd_unlock = True
				else :
					robot_.io_cmd_unlock = False
				print_vels(robot_)
			elif key == 'o':
				if robot_.io_cmd_speaker == False :
					robot_.io_cmd_speaker = True
				else :
					robot_.io_cmd_speaker = False
				print_vels(robot_)
 
			elif key == 's' :
				robot_.e_stop()
				print_vels(robot_)
			elif key == ' ' :
				if robot_.gear == 7 :
					robot_.gear =1
				else :
					robot_.gear = robot_.gear +1 
				# if robot_.mode_flag =='normal':
				# 	robot_.mode_flag ='parallel'
				# else :
				# 	robot_.mode_flag ='normal'
				# robot_.target_linear_x_vel   = 0.0
				# robot_.control_linear_x_vel  = 0.0
				# robot_.target_linear_y_vel   = 0.0
				# robot_.control_linear_y_vel  = 0.0
				# robot_.target_angular_vel  = 0.0
				# robot_.control_angular_vel = 0.0
				print("Mode change : {}".format(robot_.gear))
			elif key == '/':
				if robot_.con_idx == 5:
					robot_.con_idx = 0
				else :
					robot_.con_idx = robot_.con_idx+1 
				print('현재 모드는 {0}'.format(robot_.control_mode[robot_.con_idx]))
			

					
			twist = Twist()
			steering_cmd = SteeringCtrlCmd()
			ctrl_cmd = CtrlCmd()
			faf_cmd = FrontAngleFreeCtrlCmd()
			raf_cmd = RearAngleFreeCtrlCmd()
			fvf_cmd = FrontVelocityFreeCtrlCmd()
			rvf_cmd = RearVelocityFreeCtrlCmd()
			io_cmd = IoCmd()
			# mode = Bool()

			ctrl_cmd.ctrl_cmd_gear = robot_.gear
			steering_cmd.ctrl_cmd_gear = robot_.gear
			faf_cmd.ctrl_cmd_gear=  robot_.gear
			raf_cmd.ctrl_cmd_gear = robot_.gear
			fvf_cmd.ctrl_cmd_gear = robot_.gear
			rvf_cmd.ctrl_cmd_gear = robot_.gear		
			if robot_.gear == 6:
				# mode.data = True
				ctrl_cmd._ctrl_cmd_linear = robot_.target_linear_x_vel
				ctrl_cmd._ctrl_cmd_angular = robot_.target_angular_vel * 57.2958
				ctrl_cmd._ctrl_cmd_slipangle = 0.0
			elif robot_.gear == 7:
				# mode.data = False
				ctrl_cmd._ctrl_cmd_linear = (robot_.target_linear_x_vel**2 + robot_.target_linear_y_vel**2)**0.5
				ctrl_cmd._ctrl_cmd_angular = 0.0
				ctrl_cmd._ctrl_cmd_slipangle = math.atan2(robot_.target_linear_y_vel,robot_.target_linear_x_vel) * 57.2958
				if abs(ctrl_cmd.ctrl_cmd_slipangle) > 90 :
					ctrl_cmd._ctrl_cmd_linear * -1
					if ctrl_cmd.ctrl_cmd_slipangle > 0 :
						ctrl_cmd._ctrl_cmd_slipangle = ctrl_cmd.ctrl_cmd_slipangle -180 
					else :
						ctrl_cmd._ctrl_cmd_slipangle = ctrl_cmd.ctrl_cmd_slipangle + 180


			steering_cmd._steering_ctrl_cmd_velocity= robot_.target_steering_cmd_velocity 
			steering_cmd._steering_ctrl_cmd_steering= robot_.target_steering_cmd_steering * 57.2958
			steering_cmd._steering_ctrl_cmd_slipangle= robot_.target_steering_cmd_slipangle * 57.2958
			faf_cmd._free_ctrl_cmd_angle_lf = robot_.target_free_cmd_angle_lf * 57.2958
			faf_cmd._free_ctrl_cmd_angle_rf = robot_.target_free_cmd_angle_rf * 57.2958
			raf_cmd._free_ctrl_cmd_angle_lr = robot_.target_free_cmd_angle_lr * 57.2958
			raf_cmd._free_ctrl_cmd_angle_rr = robot_.target_free_cmd_angle_rr * 57.2958
			fvf_cmd._free_ctrl_cmd_velocity_lf = robot_.target_free_cmd_velocity_lf 
			fvf_cmd._free_ctrl_cmd_velocity_rf = robot_.target_free_cmd_velocity_rf
			rvf_cmd._free_ctrl_cmd_velocity_lr = robot_.target_free_cmd_velocity_lr
			rvf_cmd._free_ctrl_cmd_velocity_rr = robot_.target_free_cmd_velocity_rr
			io_cmd.io_cmd_unlock = robot_.io_cmd_unlock
			io_cmd.io_cmd_speaker = robot_.io_cmd_speaker
			ctrl_cmd_pub.publish(ctrl_cmd)
			# cmd_steering_pub.publish(steering_cmd)
			# faf_cmd_pub.publish(faf_cmd)
			# raf_cmd_pub.publish(raf_cmd)
			# fvf_cmd_pub.publish(fvf_cmd)
			# rvf_cmd_pub.publish(rvf_cmd)
			io_cmd_pub.publish(io_cmd)


	except Exception as e:
		print(e)

	finally:
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
		# cmd_vel_pub.publish(twist)
		
		if os.name !='nt':
			termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
			
if __name__ == '__main__':
	main()
