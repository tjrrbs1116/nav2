#ifndef __CANCONTROL_NODE_H__
#define __CANCONTROL_NODE_H__



#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "piot_can_msgs/msg/ctrl_cmd.hpp"
#include "piot_can_msgs/msg/io_cmd.hpp"
#include "piot_can_msgs/msg/ctrl_fb.hpp"
#include "piot_can_msgs/msg/lr_wheel_fb.hpp"
#include "piot_can_msgs/msg/rr_wheel_fb.hpp"
#include "piot_can_msgs/msg/lf_wheel_fb.hpp"
#include "piot_can_msgs/msg/rf_wheel_fb.hpp"
#include "piot_can_msgs/msg/io_fb.hpp"
#include "piot_can_msgs/msg/steering_ctrl_cmd.hpp"
#include "piot_can_msgs/msg/front_angle_free_ctrl_cmd.hpp"
#include "piot_can_msgs/msg/front_velocity_free_ctrl_cmd.hpp"
#include "piot_can_msgs/msg/rear_angle_free_ctrl_cmd.hpp"
#include "piot_can_msgs/msg/rear_velocity_free_ctrl_cmd.hpp"
#include "piot_can_msgs/msg/bms_fb.hpp"
#include "piot_can_msgs/msg/bms_flag_fb.hpp"
#include "piot_can_msgs/msg/steering_ctrl_fb.hpp"
#include "piot_can_msgs/msg/front_angle_fb.hpp"
#include "piot_can_msgs/msg/rear_angle_fb.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>

using namespace std::chrono_literals;
using std::placeholders::_1;


namespace piot_tool {
class CanControl : public rclcpp::Node
{
public:
	CanControl();
	~CanControl();

private:

	rclcpp::Publisher<piot_can_msgs::msg::CtrlFb>::SharedPtr ctrl_fb_pub_;
	rclcpp::Publisher<piot_can_msgs::msg::LrWheelFb>::SharedPtr lr_wheel_fb_pub_;
	rclcpp::Publisher<piot_can_msgs::msg::RrWheelFb>::SharedPtr rr_wheel_fb_pub_;
	rclcpp::Publisher<piot_can_msgs::msg::IoFb>::SharedPtr io_fb_pub_;
	rclcpp::Publisher<piot_can_msgs::msg::RfWheelFb>::SharedPtr rf_wheel_fb_pub_;
	rclcpp::Publisher<piot_can_msgs::msg::LfWheelFb>::SharedPtr lf_wheel_fb_pub_;
	rclcpp::Publisher<piot_can_msgs::msg::BmsFb>::SharedPtr bms_fb_pub_;
	rclcpp::Publisher<piot_can_msgs::msg::BmsFlagFb>::SharedPtr bms_flag_fb_pub_;
	rclcpp::Publisher<piot_can_msgs::msg::SteeringCtrlFb>::SharedPtr steering_ctrl_fb_pub_;
	rclcpp::Publisher<piot_can_msgs::msg::FrontAngleFb>::SharedPtr front_angle_fb_pub_;
	rclcpp::Publisher<piot_can_msgs::msg::RearAngleFb>::SharedPtr rear_angle_fb_pub_;

	rclcpp::Subscription<piot_can_msgs::msg::CtrlCmd>::SharedPtr ctrl_cmd_sub_;
	rclcpp::Subscription<piot_can_msgs::msg::IoCmd>::SharedPtr io_cmd_sub_;
	rclcpp::Subscription<piot_can_msgs::msg::SteeringCtrlCmd>::SharedPtr steering_ctrl_cmd_sub_;
	rclcpp::Subscription<piot_can_msgs::msg::FrontAngleFreeCtrlCmd>::SharedPtr front_angle_free_ctrl_cmd_sub_;
	rclcpp::Subscription<piot_can_msgs::msg::FrontVelocityFreeCtrlCmd>::SharedPtr front_velocity_free_ctrl_cmd_sub_;
	rclcpp::Subscription<piot_can_msgs::msg::RearAngleFreeCtrlCmd>::SharedPtr rear_angle_free_ctrl_cmd_sub_;
	rclcpp::Subscription<piot_can_msgs::msg::RearVelocityFreeCtrlCmd>::SharedPtr rear_velocity_free_ctrl_cmd_sub_;
	
	rclcpp::TimerBase::SharedPtr update_timer_;


	boost::mutex cmd_mutex_;

	unsigned char sendData_u_io_[8] = {0};
	unsigned char sendData_u_vel_[8] = {0};

	
	int dev_handler_;
	can_frame send_frames_[2];
	can_frame recv_frames_[1];


	void io_cmdCallBack(const piot_can_msgs::msg::IoCmd msg);
	void ctrl_cmdCallBack(const piot_can_msgs::msg::CtrlCmd msg);
	void steering_ctrl_cmdCallBack(const piot_can_msgs::msg::SteeringCtrlCmd msg);
	void front_angle_free_ctrl_cmdCallBack(const piot_can_msgs::msg::FrontAngleFreeCtrlCmd msg);
	void front_velocity_free_ctrl_cmdCallBack(const piot_can_msgs::msg::FrontVelocityFreeCtrlCmd msg);
	void rear_angle_free_ctrl_cmdCallBack(const piot_can_msgs::msg::RearAngleFreeCtrlCmd msg);
	void rear_velocity_free_ctrl_cmdCallBack(const piot_can_msgs::msg::RearVelocityFreeCtrlCmd msg);


	void recvData();

};

}


#endif

