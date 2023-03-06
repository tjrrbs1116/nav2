#! /usr/bin/env python
import rclpy
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import os, sys, select

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


def print_vels(target_linear_x_vel, target_linear_y_vel, target_angular_vel):
	print('currently:\tlinear vel x {0}\t linear vel y {1}\t angular vel {2} '.format(target_linear_x_vel,target_linear_y_vel,target_angular_vel))


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


def main():
	settings = None
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)

	rclpy.init()
	
	qos = QoSProfile(depth=10)
	node = rclpy.create_node('piot_teleop_node')
	
	cmd_vel_pub = node.create_publisher( Twist, 'cmd_vel', qos)
	mode_pub = node.create_publisher( Bool, 'mode', qos)
	mode = Bool()
	rate = node.create_rate(10)
	mode_flag= True
	target_linear_x_vel   = 0.0
	target_linear_y_vel   = 0.0
	target_angular_vel  = 0.0
	control_linear_x_vel  = 0.0
	control_linear_y_vel  = 0.0
	control_angular_vel = 0.0

	try:
		print(msg)
		while(1):
			key = get_key(settings)
			if mode_flag == True:
				if key == 'w' :
					target_linear_x_vel = check_linear_limit_velocity(target_linear_x_vel + LIN_VEL_STEP_SIZE)
					print_vels(target_linear_x_vel,target_linear_y_vel,target_angular_vel)
				elif key == 'x' :
					target_linear_x_vel = check_linear_limit_velocity(target_linear_x_vel - LIN_VEL_STEP_SIZE)
					print_vels(target_linear_x_vel,target_linear_y_vel,target_angular_vel)
				elif key == 'a' :
					target_angular_vel = check_angular_limit_velocity(target_angular_vel + ANG_VEL_STEP_SIZE)
					print_vels(target_linear_x_vel,target_linear_y_vel,target_angular_vel)
				elif key == 'd' :
					target_angular_vel = check_angular_limit_velocity(target_angular_vel - ANG_VEL_STEP_SIZE)
					print_vels(target_linear_x_vel,target_linear_y_vel,target_angular_vel)
				elif key == 's' :
					target_linear_x_vel   = 0.0
					control_linear_x_vel  = 0.0
					target_linear_y_vel   = 0.0
					control_linear_y_vel  = 0.0
					target_angular_vel  = 0.0
					control_angular_vel = 0.0
					print_vels(target_linear_x_vel,target_linear_y_vel,target_angular_vel)
				elif key == ' ' :
					mode_flag = False
					target_linear_x_vel   = 0.0
					control_linear_x_vel  = 0.0
					target_linear_y_vel   = 0.0
					control_linear_y_vel  = 0.0
					target_angular_vel  = 0.0
					control_angular_vel = 0.0
					print ("Mode change : Parallel mode")
				else:
					if key == '\x03':
						break
			else:
				if key == 'w' :
					target_linear_x_vel = check_linear_limit_velocity(target_linear_x_vel + LIN_VEL_STEP_SIZE)
					print_vels(target_linear_x_vel,target_linear_y_vel,target_angular_vel)
				elif key == 'x' :
					target_linear_x_vel = check_linear_limit_velocity(target_linear_x_vel - LIN_VEL_STEP_SIZE)
					print_vels(target_linear_x_vel,target_linear_y_vel,target_angular_vel)
				elif key == 'a' :
					target_linear_y_vel = check_linear_limit_velocity(target_linear_y_vel + LIN_VEL_STEP_SIZE)
					print_vels(target_linear_x_vel,target_linear_y_vel,target_angular_vel)
				elif key == 'd' :
					target_linear_y_vel = check_linear_limit_velocity(target_linear_y_vel - LIN_VEL_STEP_SIZE)
					print_vels(target_linear_x_vel,target_linear_y_vel,target_angular_vel)
				elif key == 's' :
					target_linear_x_vel   = 0.0
					control_linear_x_vel  = 0.0
					target_linear_y_vel   = 0.0
					control_linear_y_vel  = 0.0
					target_angular_vel  = 0.0
					control_angular_vel = 0.0
					print_vels(target_linear_x_vel,target_linear_y_vel,target_angular_vel)
				elif key == ' ' :
					mode_flag = True
					target_linear_x_vel   = 0.0
					control_linear_x_vel  = 0.0
					target_linear_y_vel   = 0.0
					control_linear_y_vel  = 0.0
					target_angular_vel  = 0.0
					control_angular_vel = 0.0
					print ("Mode change : Normal mode")
				else:
					if (key == '\x03'):
						break

			twist = Twist()

			control_linear_x_vel = make_simple_profile(control_linear_x_vel, target_linear_x_vel, (LIN_VEL_STEP_SIZE/2.0))
			control_linear_y_vel = make_simple_profile(control_linear_y_vel, target_linear_y_vel, (LIN_VEL_STEP_SIZE/2.0))
			twist.linear.x = control_linear_x_vel; twist.linear.y = control_linear_y_vel; twist.linear.z = 0.0

			control_angular_vel = make_simple_profile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

			cmd_vel_pub.publish(twist)
			
			mode.data = mode_flag
			mode_pub.publish(mode)

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
