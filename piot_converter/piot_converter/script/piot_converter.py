#! /usr/bin/env python
import rclpy
from rclpy.node import Node
import math
import numpy as np
from math import sin, cos
from rclpy.qos import QoSProfile
from piot_can_msgs.msg import CtrlCmd, CtrlFb
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def quaternion_from_euler(ai, aj, ak):
	ai /= 2.0
	aj /= 2.0
	ak /= 2.0
	ci = math.cos(ai)
	si = math.sin(ai)
	cj = math.cos(aj)
	sj = math.sin(aj)
	ck = math.cos(ak)
	sk = math.sin(ak)
	cc = ci*ck
	cs = ci*sk
	sc = si*ck
	ss = si*sk

	q = np.empty((4, ))
	q[0] = cj*sc - sj*cs
	q[1] = cj*ss + sj*cc
	q[2] = cj*cs - sj*sc
	q[3] = cj*cc + sj*ss

	return q

class PiotConverter(Node):

	def __init__(self):
		super().__init__('piot_converter_node')
		qos = QoSProfile(depth=10)
		self.ctrl_cmd_pub = self.create_publisher(CtrlCmd, 'ctrl_cmd', qos)
		self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos)
		self.ctrl_cmd = CtrlCmd()
		self.ctrl_cmd_gear = 1 # Gear Parking
		self.ctrl_cmd_linear = 0.0
		self.ctrl_cmd_angular = 0.0
		self.ctrl_cmd_slipangle = 0.0
	
		self.mode_sub = self.create_subscription(Bool, 'mode', self.mode_callback, qos)
		self.mode = Bool()
		self.mode_flag = True
	
		self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', qos)
		self.ctrl_fb_sub = self.create_subscription(CtrlFb, 'ctrl_fb', self.ctrl_fb_callback, qos)
		self.odom_broadcaster = TransformBroadcaster(self)
		self.x = 0.0
		self.y = 0.0
		self.th = 0.0
		self.v_x = 0.0
		self.v_y = 0.0
		self.v_th = 0.0
	
		self.current_time = self.get_clock().now().to_msg()
		self.last_time = self.get_clock().now().to_msg()

		self.timer = self.create_timer(0.01, self.timer_callback)
        
        
	def cmd_vel_callback(self, msg):
		if self.mode_flag == True:	
			self.ctrl_cmd_gear = 6 # Gear 4T4D
			self.ctrl_cmd_linear = msg.linear.x
			self.ctrl_cmd_angular = msg.angular.z * 57.2958
			self.ctrl_cmd_slipangle = 0.0

		else:
			self.ctrl_cmd_gear = 7 # Gear Parallel Moving
			self.ctrl_cmd_linear = (msg.linear.x**2 + msg.linear.y**2)**0.5
			self.ctrl_cmd_angular = 0.0
			self.ctrl_cmd_slipangle = math.atan2(msg.linear.y,msg.linear.x) * 57.2958
			if abs(self.ctrl_cmd_slipangle) > 90:
				self.ctrl_cmd_linear = self.ctrl_cmd_linear * -1
				if self.ctrl_cmd_slipangle > 0:
					self.ctrl_cmd_slipangle = self.ctrl_cmd_slipangle - 180
				else:
					self.ctrl_cmd_slipangle = self.ctrl_cmd_slipangle + 180

	def mode_callback(self, msg):
		self.mode_flag = msg.data

	def ctrl_fb_callback(self, msg):
		ctrl_fb_gear = msg.ctrl_fb_gear
		ctrl_fb_linear = msg.ctrl_fb_linear
		ctrl_fb_angular = msg.ctrl_fb_angular
		ctrl_fb_slipangle = msg.ctrl_fb_slipangle

		if ctrl_fb_gear == 6 : # Gear 4T4D
			self.v_x = ctrl_fb_linear
			self.v_y = 0.0
			self.v_th = ctrl_fb_angular * 0.0174533

		elif ctrl_fb_gear == 7 : # Gear Parallel Moving
			self.v_x = ctrl_fb_linear * cos(ctrl_fb_slipangle * 0.0174533)
			self.v_y = ctrl_fb_linear * sin(ctrl_fb_slipangle * 0.0174533)
			self.v_th = 0.0

	def timer_callback(self):
		self.ctrl_cmd.ctrl_cmd_gear = self.ctrl_cmd_gear
		self.ctrl_cmd.ctrl_cmd_linear = self.ctrl_cmd_linear
		self.ctrl_cmd.ctrl_cmd_angular = self.ctrl_cmd_angular
		self.ctrl_cmd.ctrl_cmd_slipangle = self.ctrl_cmd_slipangle
		self.ctrl_cmd_pub.publish(self.ctrl_cmd)

		self.current_time = self.get_clock().now().to_msg()
		dt = (self.current_time.sec - self.last_time.sec) + (self.current_time.nanosec/1e+9 - self.last_time.nanosec/1e+9)
		delta_x = (self.v_x * cos(self.th) - self.v_y * sin(self.th)) * dt
		delta_y = (self.v_x * sin(self.th) + self.v_y * cos(self.th)) * dt
		delta_th = self.v_th * dt

		self.x += delta_x
		self.y += delta_y
		self.th += delta_th
			
		q = quaternion_from_euler(0,0,self.th)
		odom = Odometry()			
			
		odom.header.stamp = self.current_time
		odom.header.frame_id = 'odom'
		odom.child_frame_id = 'base_link'
			
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0.0
			
		odom.pose.pose.orientation.x = q[0]
		odom.pose.pose.orientation.y = q[1]
		odom.pose.pose.orientation.z = q[2]
		odom.pose.pose.orientation.w = q[3]
			
		odom.twist.twist.linear.x = self.v_x
		odom.twist.twist.linear.y = self.v_y
		odom.twist.twist.angular.z = self.v_th
		self.odom_pub.publish(odom)

		self.last_time = self.current_time
			
			
def main(args=None):
	rclpy.init(args=args)
	piot_converter = PiotConverter()
	rclpy.spin(piot_converter)
	piot_converter.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

