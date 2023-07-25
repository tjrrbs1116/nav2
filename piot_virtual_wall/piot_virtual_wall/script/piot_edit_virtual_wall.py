#! /usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

import rclpy
import numpy as np
import cv2
import argparse
import logging
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.srv import GetMap

origin_map_dir = os.path.expanduser(os.path.join('~', 'f1.pgm'))
keepout_map_dir = os.path.expanduser(os.path.join('~', 'keepout_mask.pgm'))
origin_yaml_dir = os.path.expanduser(os.path.join('~', 'f1.yaml'))
keepout_yaml_dir = os.path.expanduser(os.path.join('~', 'keepout_mask.yaml'))

Image = cv2.imread(origin_map_dir,-1)
#Image = cv2.rotate(Image, cv2.ROTATE_90_COUNTERCLOCKWISE)




count = 0

rviz_x_1 = 0.0
rviz_x_2 = 0.0
rviz_y_1 = 0.0
rviz_y_2 = 0.0

img_x_1 = 0.0
img_x_2 = 0.0
img_y_1 = 0.0
img_y_2 = 0.0

map_width  = 0.0
map_height = 0.0

origin_x = 0.0
origin_y = 0.0

marker_count = 0

class PiotEditVirtualWall(Node):
	def __init__(self):
		super().__init__('piot_edit_virtual_wall_node')
		qos = QoSProfile(depth=10)
		self.point_sub = self.create_subscription(PointStamped, 'clicked_point', self.point_callback, qos)
		self.marker_pub = self.create_publisher(Marker,'marker',qos)   
    
		self.map_cli  = self.create_client(GetMap, 'map_server/map')
		self.map_req = GetMap.Request()
		while not self.map_cli.wait_for_service(timeout_sec=5.0):
			self.get_logger().info('service not available,waiting again') 
		
		self.send_request()     
   
	def point_callback(self,msg):
		global Image, count, rviz_x_1, rviz_x_2, rviz_y_1, rviz_y_2, img_x_1, img_x_2, img_y_1, img_y_2, marker_count
		height, width = Image.shape
		count = count +1
		if count%2 == 1:   
			rviz_x_1 = msg.point.x
			rviz_y_1 = msg.point.y
			self.get_logger().info('x = {0}    y = {1}'.format(rviz_x_1,rviz_y_1))
			marker = Marker()
			marker.header.frame_id = 'map'
			marker.type = marker.SPHERE
			marker.action = marker.ADD
			marker.scale.x = 0.3
			marker.scale.y = 0.3
			marker.scale.z = 0.3
			marker.color.a = 1.0
			marker.color.r = 1.0
			marker.color.g = 0.0
			marker.color.b = 0.0
			marker.pose.position.x = rviz_x_1
			marker.pose.position.y = rviz_y_1
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			marker.id = marker_count
			self.marker_pub.publish(marker)
			marker_count = marker_count + 1    
#			img_y_1 = int(width * (1-(msg.point.x - origin_x)/(map_width/20)))
#			img_x_1 = int(height  * (1-(msg.point.y - origin_y)/(map_height/20)))
			img_y_1 = int(height - (msg.point.y - origin_y)/0.05)
			img_x_1 = int((msg.point.x - origin_x) / 0.05)
		else:
			rviz_x_2 = msg.point.x
			rviz_y_2 = msg.point.y
			self.get_logger().info('x = {0}    y = {1}'.format(rviz_x_2,rviz_y_2))
			marker = Marker()
			marker.header.frame_id = 'map'
			marker.type = marker.SPHERE
			marker.action = marker.ADD
			marker.scale.x = 0.3
			marker.scale.y = 0.3
			marker.scale.z = 0.3
			marker.color.a = 1.0
			marker.color.r = 1.0
			marker.color.g = 0.0
			marker.color.b = 0.0
			marker.pose.position.x = rviz_x_2
			marker.pose.position.y = rviz_y_2
			marker.pose.orientation.z = 0.0
			marker.pose.orientation.w = 1.0
			marker.id = marker_count
			self.marker_pub.publish(marker)
			marker_count = marker_count + 1
      
			line_marker = Marker()
			line_marker.header.frame_id = 'map'
			line_marker.type = marker.LINE_STRIP
			line_marker.action = marker.ADD
			line_marker.scale.x = 0.2
			line_marker.color.a = 1.0
			line_marker.color.r = 0.0
			line_marker.color.g = 1.0
			line_marker.color.b = 0.0
			line_marker.points = []
			first_line_point = Point()
			first_line_point.x = rviz_x_1
			first_line_point.y = rviz_y_1
			line_marker.points.append(first_line_point)
			second_line_point = Point()
			second_line_point.x = rviz_x_2
			second_line_point.y = rviz_y_2
			line_marker.points.append(second_line_point)
			line_marker.id = marker_count
			self.marker_pub.publish(line_marker)
			marker_count = marker_count + 1
      
#			img_y_2 = int(width * (1-(msg.point.x - origin_x)/(map_width/20)))
#			img_x_2 = int(height  * (1-(msg.point.y - origin_y)/(map_height/20)))
			img_y_2 = int(height - (msg.point.y - origin_y)/0.05)
			img_x_2 = int((msg.point.x - origin_x) / 0.05)
			cv2.line(Image,(img_x_1,img_y_1),(img_x_2,img_y_2),(0,0,0),2)
#			SaveImage = cv2.rotate(Image, cv2.ROTATE_90_CLOCKWISE)
			SaveImage = Image      
			cv2.imwrite(keepout_map_dir,SaveImage)
      
			with open(origin_yaml_dir) as f:
				yaml_data = yaml.load(f, Loader=yaml.FullLoader)

			yaml_data['image']='keepout_mask.pgm'
      
			with open(keepout_yaml_dir,'w') as f:
				yaml.dump(yaml_data, f, default_flow_style=None, sort_keys=None)
      
	def send_request(self):
		global map_width, map_height, origin_x, origin_y
		wait = self.map_cli.call_async(self.map_req)
		rclpy.spin_until_future_complete(self,wait)
		response = wait.result()
		if response is not None:
			map_width    = response.map.info.width
			map_height   = response.map.info.height
			origin_x     = response.map.info.origin.position.x
			origin_y     = response.map.info.origin.position.y
			self.get_logger().info('{0} {1}'.format(origin_x,origin_y))      
		else:
			self.get_logger().info('Request Failed')    			
			
def main(): 
	rclpy.init()
	piot_edit_virtual_wall = PiotEditVirtualWall()
	rclpy.spin(piot_edit_virtual_wall)
	piot_edit_virtual_wall.destroy_node()
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()