#! /usr/bin/env python
import rclpy
from rclpy.node import Node
import math
import numpy as np
from math import sin, cos
from rclpy.qos import QoSProfile
from piot_can_msgs.msg import CtrlCmd, CtrlFb
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster

def euler_from_quaternion(x, y, z, w):

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	roll_x = math.atan2(t0, t1)

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch_y = math.asin(t2)

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw_z = math.atan2(t3, t4)

	return roll_x, pitch_y, yaw_z # in radians

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
		self.log_cmd_pub  = self.create_publisher(CtrlCmd, 'log_cmd', qos)

		self.imu_calibration_flag = False
		#cmd vel target data - before pid controller
		self.cmd_vel = Twist()


		#real robot target data - after pid controller
		self.ctrl_cmd = CtrlCmd()
		self.old_ctrl_cmd = CtrlCmd()
		self.ctrl_cmd_gear = 1 # Gear Parking
		self.ctrl_cmd_linear = 0.0
		self.ctrl_cmd_angular = 0.0
		self.ctrl_cmd_slipangle = 0.0

		self.log =CtrlCmd()
		self.log.ctrl_cmd_angular =0.0

    ##Linear PID Gain
		self.P_gain = 0.08
		self.I_gain = 1.6
		self.D_gain = 0.03
		##Angular PID Gain
		self.P_a_gain = 0.8 # 0.8
		self.I_a_gain = 0.6
		self.D_a_gain = 0.03
		self.target_linear_p_term = 0.0
		self.target_linear_i_term = 0.0
		self.target_linear_d_term = 0.0
		self.target_angular_p_term = 0.0
		self.target_angular_i_term = 0.0
		self.target_angular_d_term = 0.0

		self.target_linear = 0.0
		self.target_angular = 0.0
		self.linear_error = 0.0
		self.angular_error =0.0
		#controller frequency
		self.n_t = self.get_clock().now().to_msg()
		self.o_t = self.get_clock().now().to_msg()
		self.first_flag = False
		self.old_linear_error = 0.0
		self.old_angular_error = 0.0


		self.fb = CtrlFb()
		self.mode_flag = True

#		self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
		self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', qos)
		#self.odom2_pub = self.create_publisher(Odometry, '/notfb_odom' ,qos)
		# self.imu_sub = self.create_subscription(Imu, '/imu_sensing',self.imu_sub_callback,qos)
		self.ctrl_fb_sub = self.create_subscription(CtrlFb, 'ctrl_fb', self.ctrl_fb_callback, qos)
#		self.odom_broadcaster = TransformBroadcaster(self)
		self.x = 0.0
		self.y = 0.0
		self.th = 0.0
		self.v_x = 0.0
		self.v_y = 0.0
		self.v_th = 0.0

		## not fb odom data
		self.nf_x = 0.0
		self.nf_y = 0.0
		self.nf_th = 0.0
		self.nf_v_x = 0.0
		self.nf_v_y = 0.0
		self.nf_v_th = 0.0

		self.current_time = self.get_clock().now().to_msg()
		self.last_time = self.get_clock().now().to_msg()
		self.timer = self.create_timer(0.01, self.timer_callback)

	# def imu_sub_callback(self,msg):
	# 	if(self.imu_calibration_flag == False):
	# 		euler = euler_from_quaternion(msg.orientation.x , msg.orientation.y , msg.orientation.z , msg.orientation.w)
	# 		self.th = euler[2] #radian th
	# 		self.imu_calibration_flag = True

	def cmd_vel_callback(self, msg):
#		if self.mode_flag == True:
#			self.ctrl_cmd_gear = 6 # Gear 4T4D
#			self.ctrl_cmd_linear = msg.linear.x
#			self.ctrl_cmd_angular = msg.angular.z * 57.2958
#			self.ctrl_cmd_slipangle = 0.0

#		else:
#			self.ctrl_cmd_gear = 7 # Gear Parallel Moving
#			self.ctrl_cmd_linear = (msg.linear.x**2 + msg.linear.y**2)**0.5
#			self.ctrl_cmd_angular = 0.0
#			self.ctrl_cmd_slipangle = math.atan2(msg.linear.y,msg.linear.x) * 57.2958
#			if abs(self.ctrl_cmd_slipangle) > 90:
#				self.ctrl_cmd_linear = self.ctrl_cmd_linear * -1
#				if self.ctrl_cmd_slipangle > 0:
#					self.ctrl_cmd_slipangle = self.ctrl_cmd_slipangle - 180
#				else:
#					self.ctrl_cmd_slipangle = self.ctrl_cmd_slipangle + 180

		if self.mode_flag == True:
			self.ctrl_cmd_gear = 6

		if abs(msg.linear.y) > 0:
			self.mode_flag = False
			self.ctrl_cmd_gear = 7 # Gear Parallel Moving
			self.ctrl_cmd_linear = (msg.linear.x**2 + msg.linear.y**2)**0.5
			self.ctrl_cmd_angular = 0.0
			self.ctrl_cmd_slipangle = math.atan2(msg.linear.y,msg.linear.x) * 57.2958

			if self.ctrl_cmd_slipangle > 85.0 and self.ctrl_cmd_slipangle < 95.0:
				self.ctrl_cmd_slipangle = 90.0
			if self.ctrl_cmd_slipangle < -85.0 and self.ctrl_cmd_slipangle > -95.0:
				self.ctrl_cmd_slipangle = -90.0

			if abs(self.ctrl_cmd_slipangle) > 90:
				self.ctrl_cmd_linear = self.ctrl_cmd_linear * -1
				if self.ctrl_cmd_slipangle > 0:
					self.ctrl_cmd_slipangle = self.ctrl_cmd_slipangle - 180
				else:
					self.ctrl_cmd_slipangle = self.ctrl_cmd_slipangle + 180



		self.cmd_vel = msg


		self.ctrl_cmd.ctrl_cmd_gear = self.ctrl_cmd_gear
		self.ctrl_cmd.ctrl_cmd_linear = self.target_linear
		self.ctrl_cmd.ctrl_cmd_angular = self.target_angular * 57.2958 # degree target
		self.ctrl_cmd.ctrl_cmd_slipangle = 0.0

		self.ctrl_cmd_pub.publish(self.ctrl_cmd)
		self.log_cmd_pub.publish(self.log)

		self.old_ctrl_cmd = self.ctrl_cmd


	def ctrl_fb_callback(self, msg):

		self.fb.ctrl_fb_gear = msg.ctrl_fb_gear
		self.fb.ctrl_fb_linear = msg.ctrl_fb_linear
		self.fb.ctrl_fb_angular = msg.ctrl_fb_angular
		self.fb.ctrl_fb_slipangle = msg.ctrl_fb_slipangle

		if self.fb.ctrl_fb_gear == 6 : # Gear 4T4D
			self.v_x = self.fb.ctrl_fb_linear
			self.v_y = 0.0
			self.v_th = self.fb.ctrl_fb_angular * 0.0174533 #deg2rad

			# self.nf_v_x = self.fb.ctrl_fb_linear
			# self.nf_v_y = 0.0
			# self.nf_v_th = self.ctrl_cmd.ctrl_cmd_angular * 0.0174533


		elif self.fb.ctrl_fb_gear == 7 : # Gear Parallel Moving
			self.v_x = self.fb.ctrl_fb_linear * cos(self.fb.ctrl_fb_slipangle * 0.0174533)
			self.v_y = self.fb.ctrl_fb_linear * sin(self.fb.ctrl_fb_slipangle * 0.0174533)
			self.v_th = 0.0

		else:
			self.v_x = 0.0
			self.v_y = 0.0
			self.v_th = 0.0


	def timer_callback(self):



		self.current_time = self.get_clock().now().to_msg()
		dt = (self.current_time.sec - self.last_time.sec) + (self.current_time.nanosec/1e+9 - self.last_time.nanosec/1e+9)


	#--------------------------------PID------------------------------------------------#
		if self.fb.ctrl_fb_gear ==6:
			#self.linear_error = self.cmd_vel.linear.x - self.fb.ctrl_fb_linear
			#self.angular_error = self.cmd_vel.angular.z - (self.fb.ctrl_fb_angular * 0.0174533) # radian_error
			self.linear_error = self.cmd_vel.linear.x - self.old_ctrl_cmd.ctrl_cmd_linear
			self.angular_error = self.cmd_vel.angular.z - (self.old_ctrl_cmd.ctrl_cmd_angular * 0.01745333)
			##threshold
			if self.linear_error > 0.3 :
				self.linear_error = 0.3

			if self.linear_error < -0.3 :
				self.linear_error = -0.3

			if self.angular_error > 0.3 :
				self.angular_error  = 0.3

			if self.angular_error < -0.3 :
				self.angular_error = -0.3

			# self.target_linear_p_term += self.P_gain*(linear_error)
			self.target_linear_p_term = self.P_gain*(self.linear_error)
			self.target_linear_i_term += self.I_gain*(self.linear_error)*dt
			self.target_linear_d_term = self.D_gain*((self.linear_error - self.old_linear_error) / dt)
			self.target_angular_p_term = self.P_a_gain*(self.angular_error) # radian_target
			self.target_angular_i_term  += self.I_a_gain*(self.angular_error)*dt
			self.target_angular_d_term = self.D_a_gain*((self.angular_error - self.old_angular_error) / dt)

			self.target_linear = self.target_linear_p_term + self.target_linear_i_term #+ self.target_linear_d_term  #self.cmd_vel.linear.x
			#self.target_angular = self.cmd_vel.angular.z #self.target_angular_p_term + self.target_angular_i_term #+ self.target_angular_d_term
			self.target_angular = self.cmd_vel.angular.z#self.target_angular_p_term + self.target_angular_i_term #self.cmd_vel.angular.z
			self.log.ctrl_cmd_angular = self.target_angular # radian_target



			#self.o_t = self.n_t
			# self.old_linear_error = linear_error
			# self.old_angular_error = angular_error

		elif self.fb.ctrl_fb_gear == 1 :
			self.target_linear =0.0
			self.target_angular =0.0
			self.target_linear_i_term =0.0
			self.target_linear_p_term = 0.0
			self.target_angular_p_term = 0.0
			self.target_angular_i_term = 0.0
			self.linear_error =0.0
			self.angular_error =0.0
			self.old_ctrl_cmd.ctrl_cmd_angular= 0.0
			self.old_ctrl_cmd.ctrl_cmd_linear = 0.0


#-----------------------------------------------------------------------------------#

		delta_x = (self.v_x * cos(self.th) - self.v_y * sin(self.th)) * dt
		delta_y = (self.v_x * sin(self.th) + self.v_y * cos(self.th)) * dt
		delta_th = self.v_th * dt

		# delta_nf_x = (self.nf_v_x * cos(self.nf_th) - self.nf_v_y * sin(self.nf_th)) * dt
		# delta_nf_y = (self.nf_v_x * sin(self.nf_th) + self.nf_v_y * cos(self.nf_th)) * dt
		# delta_nf_th = self.nf_v_th * dt

		# self.nf_x += delta_nf_x
		# self.nf_y += delta_nf_y
		# self.nf_th += delta_nf_th

		self.x += delta_x
		self.y += delta_y
		self.th += delta_th
		# self.get_logger().info("x is : %.4f" % self.x)
		# self.get_logger().info("y is : %.4f" % self.y)
		# q1 =quaternion_from_euler(0,0,self.nf_th)


		# odom_nf =Odometry()
		# odom_nf.pose.pose.position.x = self.nf_x
		# odom_nf.pose.pose.position.y = self.nf_y
		# odom_nf.pose.pose.position.z = 0.0
		# odom_nf.pose.pose.orientation.x = q1[0]
		# odom_nf.pose.pose.orientation.y = q1[1]
		# odom_nf.pose.pose.orientation.z = q1[2]
		# odom_nf.pose.pose.orientation.w = q1[3]



		#-----------------------------------------------#
		q = quaternion_from_euler(0,0,self.th)
		odom_stamp = TransformStamped()
		odom = Odometry()

		odom.header.stamp = self.current_time
		odom.header.frame_id = 'odom'
		odom.child_frame_id = 'base_footprint'

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

		# self.odom2_pub.publish(odom_nf)
		self.odom_pub.publish(odom)
		self.last_time = self.current_time
		self.old_linear_error = self.linear_error
		self.old_angular_error = self.angular_error
		# self.get_logger().info("dt is : %.4f" % dt)

		# self.ctrl_cmd_pub.publish(self.ctrl_cmd)
		# self.log_cmd_pub.publish(self.log)






def main(args=None):
	rclpy.init(args=args)
	piot_converter = PiotConverter()
	rclpy.spin(piot_converter)
	piot_converter.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
