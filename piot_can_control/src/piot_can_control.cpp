#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <pthread.h>
#include <time.h>


#include <boost/bind.hpp>
#include <boost/thread.hpp>


#include "piot_can_control.hpp"


namespace piot_tool {
CanControl::CanControl():Node("piot_can_control_node")
{
	auto qos = rclcpp::QoS(rclcpp::KeepLast(5));
	
	ctrl_fb_pub_ = this->create_publisher<piot_can_msgs::msg::CtrlFb>("ctrl_fb", qos); 
	lr_wheel_fb_pub_ = this->create_publisher<piot_can_msgs::msg::LrWheelFb>("lr_wheel_fb", qos);
	rr_wheel_fb_pub_ = this->create_publisher<piot_can_msgs::msg::RrWheelFb>("rr_wheel_fb", qos);
	io_fb_pub_ = this->create_publisher<piot_can_msgs::msg::IoFb>("io_fb", qos);
	rf_wheel_fb_pub_ = this->create_publisher<piot_can_msgs::msg::RfWheelFb>("rf_wheel_fb", qos);
	lf_wheel_fb_pub_ = this->create_publisher<piot_can_msgs::msg::LfWheelFb>("lf_wheel_fb", qos);
	bms_fb_pub_ = this->create_publisher<piot_can_msgs::msg::BmsFb>("bms_fb", qos);
	bms_flag_fb_pub_ = this->create_publisher<piot_can_msgs::msg::BmsFlagFb>("bms_flag_fb", qos);
	steering_ctrl_fb_pub_ = this->create_publisher<piot_can_msgs::msg::SteeringCtrlFb>("steering_ctrl_fb", qos);
 	front_angle_fb_pub_ = this->create_publisher<piot_can_msgs::msg::FrontAngleFb>("front_angle_fb", qos);
	rear_angle_fb_pub_ = this->create_publisher<piot_can_msgs::msg::RearAngleFb>("rear_angle_fb", qos);
	
	ctrl_cmd_sub_ = this->create_subscription<piot_can_msgs::msg::CtrlCmd>("ctrl_cmd" ,5,std::bind(&CanControl::ctrl_cmdCallBack,this,std::placeholders::_1));
	io_cmd_sub_ = this->create_subscription<piot_can_msgs::msg::IoCmd>("io_cmd",5,std::bind(&CanControl::io_cmdCallBack,this,std::placeholders::_1));
	steering_ctrl_cmd_sub_ = this->create_subscription<piot_can_msgs::msg::SteeringCtrlCmd>("steering_ctrl_cmd",5,std::bind(&CanControl::steering_ctrl_cmdCallBack,this,std::placeholders::_1));
	front_angle_free_ctrl_cmd_sub_ = this->create_subscription<piot_can_msgs::msg::FrontAngleFreeCtrlCmd>("front_angle_free_ctrl_cmd",5,std::bind(&CanControl::front_angle_free_ctrl_cmdCallBack,this,std::placeholders::_1));
	front_velocity_free_ctrl_cmd_sub_ = this->create_subscription<piot_can_msgs::msg::FrontVelocityFreeCtrlCmd>("front_velocity_free_ctrl_cmd",5,std::bind(&CanControl::front_velocity_free_ctrl_cmdCallBack,this,std::placeholders::_1));
	rear_angle_free_ctrl_cmd_sub_ = this->create_subscription<piot_can_msgs::msg::RearAngleFreeCtrlCmd>("rear_angle_free_ctrl_cmd",5,std::bind(&CanControl::rear_angle_free_ctrl_cmdCallBack,this,std::placeholders::_1));
	rear_velocity_free_ctrl_cmd_sub_ = this->create_subscription<piot_can_msgs::msg::RearVelocityFreeCtrlCmd>("rear_velocity_free_ctrl_cmd",5,std::bind(&CanControl::rear_velocity_free_ctrl_cmdCallBack,this,std::placeholders::_1));
	
	dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (dev_handler_ < 0)
	{
		RCLCPP_ERROR(this->get_logger(),">>open can deivce error!");
		return;
	}
    else
	{
		RCLCPP_INFO(this->get_logger(),">>open can deivce success!");
	}


	struct ifreq ifr;
	
	std::string can_name("can0");

	strcpy(ifr.ifr_name,can_name.c_str());

	ioctl(dev_handler_,SIOCGIFINDEX, &ifr);



	struct sockaddr_can addr;
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	int ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),sizeof(addr));
	if (ret < 0) 
	{
		RCLCPP_ERROR(this->get_logger(),">>bind dev_handler error!\r\n");
		return;
	}
	
	update_timer_ = this->create_wall_timer(1ms, std::bind(&CanControl::recvData,this));
	    
}


CanControl::~CanControl()
{
	close(dev_handler_);
}


void CanControl::io_cmdCallBack(const piot_can_msgs::msg::IoCmd msg)
{
	static unsigned char count_1 = 0;

	cmd_mutex_.lock();

	memset(sendData_u_io_,0,8);

	sendData_u_io_[0] = 0xff;
	if(msg.io_cmd_lamp_ctrl)
		sendData_u_io_[0] &= 0xff;
	else sendData_u_io_[0] &= 0xfe;
	if(msg.io_cmd_unlock)
		sendData_u_io_[0] &= 0xff;
	else sendData_u_io_[0] &= 0xfd;

	sendData_u_io_[1] = 0xff;
	if(msg.io_cmd_lower_beam_headlamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xfe;
	if(msg.io_cmd_upper_beam_headlamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xfd;

	if(msg.io_cmd_turn_lamp == 0)
		sendData_u_io_[1] &= 0xf3;
	if(msg.io_cmd_turn_lamp == 1)
		sendData_u_io_[1] &= 0xf7;
	if(msg.io_cmd_turn_lamp == 2)
		sendData_u_io_[1] &= 0xfb;

	if(msg.io_cmd_braking_lamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xef;
	if(msg.io_cmd_clearance_lamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xdf;
	if(msg.io_cmd_fog_lamp)
		sendData_u_io_[1] &= 0xff;
	else sendData_u_io_[1] &= 0xbf;

	sendData_u_io_[2] = msg.io_cmd_speaker;

	sendData_u_io_[3] = 0;
	sendData_u_io_[4] = 0;
	sendData_u_io_[5] = 0;

	count_1 ++;
	if(count_1 == 16)	count_1 = 0;

	sendData_u_io_[6] =  count_1 << 4;

	sendData_u_io_[7] = sendData_u_io_[0] ^ sendData_u_io_[1] ^ sendData_u_io_[2] ^ sendData_u_io_[3] ^ sendData_u_io_[4] ^ sendData_u_io_[5] ^ sendData_u_io_[6];

	send_frames_[0].can_id = 0x98C4D7D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_io_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      RCLCPP_ERROR(this->get_logger(),"send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();
}


void CanControl::ctrl_cmdCallBack(const piot_can_msgs::msg::CtrlCmd msg)
{
	short linear = msg.ctrl_cmd_linear * 1000;
	short angular = msg.ctrl_cmd_angular * 100;
	short slipangle = msg.ctrl_cmd_slipangle * 100;
	static unsigned char count = 0;

	cmd_mutex_.lock();

	memset(sendData_u_vel_,0,8);

	sendData_u_vel_[0] = sendData_u_vel_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_vel_[0] = sendData_u_vel_[0] | (0xf0 & ((linear & 0x0f) << 4));

	sendData_u_vel_[1] = (linear >> 4) & 0xff;

	sendData_u_vel_[2] = sendData_u_vel_[2] | (0x0f & (linear >> 12));


	sendData_u_vel_[2] = sendData_u_vel_[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_vel_[3] = (angular >> 4) & 0xff;

	sendData_u_vel_[4] = sendData_u_vel_[4] | (0x0f & (angular >> 12));

	
	sendData_u_vel_[4] = sendData_u_vel_[4] | (0xf0 & ((slipangle & 0x0f) << 4));

	sendData_u_vel_[5] = (slipangle >> 4) & 0xff;

	sendData_u_vel_[6] = sendData_u_vel_[6] | (0x0f & (slipangle >> 12));


	count ++;

	if(count == 16)	count = 0;

	sendData_u_vel_[6] =  sendData_u_vel_[6] | (count << 4);
	

	sendData_u_vel_[7] = sendData_u_vel_[0] ^ sendData_u_vel_[1] ^ sendData_u_vel_[2] ^ sendData_u_vel_[3] ^ sendData_u_vel_[4] ^ sendData_u_vel_[5] ^ sendData_u_vel_[6];

	send_frames_[0].can_id = 0x98C4D1D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_vel_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      RCLCPP_ERROR(this->get_logger(),"send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();
}


void CanControl::steering_ctrl_cmdCallBack(const piot_can_msgs::msg::SteeringCtrlCmd msg)
{
	short linear = msg.steering_ctrl_cmd_velocity * 1000;
	short angular = msg.steering_ctrl_cmd_steering * 100;
	short slipangle = msg.steering_ctrl_cmd_slipangle * 100;
	static unsigned char count_2 = 0;

	unsigned char sendData_u_tem_[8] = {0};

	cmd_mutex_.lock();

	sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((linear & 0x0f) << 4));

	sendData_u_tem_[1] = (linear >> 4) & 0xff;

	sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (linear >> 12));


	sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((angular & 0x0f) << 4));

	sendData_u_tem_[3] = (angular >> 4) & 0xff;

	sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (angular >> 12));

	
	sendData_u_tem_[4] = sendData_u_tem_[4] | (0xf0 & ((slipangle & 0x0f) << 4));

	sendData_u_tem_[5] = (slipangle >> 4) & 0xff;

	sendData_u_tem_[6] = sendData_u_tem_[6] | (0x0f & (slipangle >> 12));


	count_2 ++;

	if(count_2 == 16)	count_2 = 0;

	sendData_u_tem_[6] =  count_2 << 4;
	

	sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];


	send_frames_[0].can_id = 0x98C4D2D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_tem_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      RCLCPP_ERROR(this->get_logger(),"send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();
}


void CanControl::front_angle_free_ctrl_cmdCallBack(const piot_can_msgs::msg::FrontAngleFreeCtrlCmd msg)
{
	short angularl = msg.free_ctrl_cmd_angle_lf * 100;
	short angularr = msg.free_ctrl_cmd_angle_rf * 100;
	static unsigned char count_4 = 0;

	unsigned char sendData_u_tem_[8] = {0};

	cmd_mutex_.lock();

	sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((angularl & 0x0f) << 4));

	sendData_u_tem_[1] = (angularl >> 4) & 0xff;

	sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (angularl >> 12));


	sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((angularr & 0x0f) << 4));

	sendData_u_tem_[3] = (angularr >> 4) & 0xff;

	sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (angularr >> 12));


	count_4 ++;

	if(count_4 == 16)	count_4 = 0;

	sendData_u_tem_[6] =  count_4 << 4;
	

	sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

	send_frames_[0].can_id = 0x98C4D5D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_tem_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      RCLCPP_ERROR(this->get_logger(),"send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();

}


void CanControl::front_velocity_free_ctrl_cmdCallBack(const piot_can_msgs::msg::FrontVelocityFreeCtrlCmd msg)
{
	short linearl = msg.free_ctrl_cmd_velocity_lf * 1000;
	short linearr = msg.free_ctrl_cmd_velocity_rf * 1000;
	static unsigned char count_3 = 0;

	unsigned char sendData_u_tem_[8] = {0};

	cmd_mutex_.lock();

	sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((linearl & 0x0f) << 4));

	sendData_u_tem_[1] = (linearl >> 4) & 0xff;

	sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (linearl >> 12));


	sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((linearr & 0x0f) << 4));

	sendData_u_tem_[3] = (linearr >> 4) & 0xff;

	sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (linearr >> 12));

	count_3 ++;

	if(count_3 == 16)	count_3 = 0;

	sendData_u_tem_[6] =  count_3 << 4;
	

	sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

	send_frames_[0].can_id = 0x98C4D3D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_tem_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      RCLCPP_ERROR(this->get_logger(),"send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();
}


void CanControl::rear_angle_free_ctrl_cmdCallBack(const piot_can_msgs::msg::RearAngleFreeCtrlCmd msg)
{
	short angularl = msg.free_ctrl_cmd_angle_lr * 100;
	short angularr = msg.free_ctrl_cmd_angle_rr * 100;
	static unsigned char count_6 = 0;

	unsigned char sendData_u_tem_[8] = {0};

	cmd_mutex_.lock();

	sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((angularl & 0x0f) << 4));

	sendData_u_tem_[1] = (angularl >> 4) & 0xff;

	sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (angularl >> 12));


	sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((angularr & 0x0f) << 4));

	sendData_u_tem_[3] = (angularr >> 4) & 0xff;

	sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (angularr >> 12));


	count_6 ++;

	if(count_6 == 16)	count_6 = 0;

	sendData_u_tem_[6] =  count_6 << 4;
	

	sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

	send_frames_[0].can_id = 0x98C4D6D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_tem_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      RCLCPP_ERROR(this->get_logger(),"send message failed, error code: %d",ret);
    }

	cmd_mutex_.unlock();

}


void CanControl::rear_velocity_free_ctrl_cmdCallBack(const piot_can_msgs::msg::RearVelocityFreeCtrlCmd msg)
{
	short linearl = msg.free_ctrl_cmd_velocity_lr * 1000;
	short linearr = msg.free_ctrl_cmd_velocity_rr * 1000;
	static unsigned char count_5 = 0;

	unsigned char sendData_u_tem_[8] = {0};

	cmd_mutex_.lock();

	sendData_u_tem_[0] = sendData_u_tem_[0] | (0x0f & msg.ctrl_cmd_gear);
	
	sendData_u_tem_[0] = sendData_u_tem_[0] | (0xf0 & ((linearl & 0x0f) << 4));

	sendData_u_tem_[1] = (linearl >> 4) & 0xff;

	sendData_u_tem_[2] = sendData_u_tem_[2] | (0x0f & (linearl >> 12));


	sendData_u_tem_[2] = sendData_u_tem_[2] | (0xf0 & ((linearr & 0x0f) << 4));

	sendData_u_tem_[3] = (linearr >> 4) & 0xff;

	sendData_u_tem_[4] = sendData_u_tem_[4] | (0x0f & (linearr >> 12));

	count_5 ++;

	if(count_5 == 16)	count_5 = 0;

	sendData_u_tem_[6] =  count_5 << 4;
	

	sendData_u_tem_[7] = sendData_u_tem_[0] ^ sendData_u_tem_[1] ^ sendData_u_tem_[2] ^ sendData_u_tem_[3] ^ sendData_u_tem_[4] ^ sendData_u_tem_[5] ^ sendData_u_tem_[6];

	send_frames_[0].can_id = 0x98C4D4D0;
    send_frames_[0].can_dlc = 8;

	memcpy(send_frames_[0].data, sendData_u_tem_, 8);

	int ret = write(dev_handler_, &send_frames_[0], sizeof(send_frames_[0]));
    if (ret <= 0) 
	{
      RCLCPP_ERROR(this->get_logger(),"send message failed, error code: %d",ret);
    }
	

	cmd_mutex_.unlock();

}


void CanControl::recvData()
{

	if(read(dev_handler_, &recv_frames_[0], sizeof(recv_frames_[0])) >= 0)
	{
		for(int j=0;j<1;j++)
		{
			switch (recv_frames_[0].can_id)
			{
				case 0x98C4D1EF:
				{
					piot_can_msgs::msg::CtrlFb msg;
					msg.ctrl_fb_gear = 0x0f & recv_frames_[0].data[0];
					
					msg.ctrl_fb_linear = (float)((short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;
					
					msg.ctrl_fb_angular = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;

					msg.ctrl_fb_slipangle = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 12 | recv_frames_[0].data[5] << 4 | (recv_frames_[0].data[4] & 0xf0) >> 4)) / 100;
						

					unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

					if(crc == recv_frames_[0].data[7])
					{
								
						ctrl_fb_pub_->publish(msg);
					}

					break;
				}

				case 0x98C4D2EF:
				{
					piot_can_msgs::msg::SteeringCtrlFb msg;
					msg.steering_ctrl_fb_gear = 0x0f & recv_frames_[0].data[0];
						
					msg.steering_ctrl_fb_velocity = (float)((short)((recv_frames_[0].data[2] & 0x0f) << 12 | recv_frames_[0].data[1] << 4 | (recv_frames_[0].data[0] & 0xf0) >> 4)) / 1000;
					
					msg.steering_ctrl_fb_steering = (float)((short)((recv_frames_[0].data[4] & 0x0f) << 12 | recv_frames_[0].data[3] << 4 | (recv_frames_[0].data[2] & 0xf0) >> 4)) / 100;

					msg.steering_ctrl_fb_slipangle = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 12 | recv_frames_[0].data[5] << 4 | (recv_frames_[0].data[4] & 0xf0) >> 4)) / 100;
						

					unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

					if(crc == recv_frames_[0].data[7])
					{
								
						steering_ctrl_fb_pub_->publish(msg);
					}

					break;
				}

				case 0x98C4D6EF:
				{
					piot_can_msgs::msg::LfWheelFb msg;
					msg.lf_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;
	
					msg.lf_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

					unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

					if(crc == recv_frames_[0].data[7])
					{
								
						lf_wheel_fb_pub_->publish(msg);
					}

					break;
				}

				case 0x98C4D7EF:
				{
					piot_can_msgs::msg::LrWheelFb msg;
					msg.lr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

					msg.lr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

					unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

					if(crc == recv_frames_[0].data[7])
					{
								
						lr_wheel_fb_pub_->publish(msg);
					}

					break;
				}

				case 0x98C4D8EF:
				{
					piot_can_msgs::msg::RrWheelFb msg;
					msg.rr_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

					msg.rr_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

					unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

					if(crc == recv_frames_[0].data[7])
					{
								
						rr_wheel_fb_pub_->publish(msg);
					}

					break;
				}

				case 0x98C4D9EF:
				{
					piot_can_msgs::msg::RfWheelFb msg;
					msg.rf_wheel_fb_velocity = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 1000;

					msg.rf_wheel_fb_pulse = (int)(recv_frames_[0].data[5] << 24 | recv_frames_[0].data[4] << 16 | recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2]);

					unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

					if(crc == recv_frames_[0].data[7])
					{
								
						rf_wheel_fb_pub_->publish(msg);
					}

					break;
				}

				case 0x98C4DAEF:
				{
					piot_can_msgs::msg::IoFb msg;
					if(0x01 & recv_frames_[0].data[0]) msg.io_fb_lamp_ctrl = true;	else msg.io_fb_lamp_ctrl = false;
	
					if(0x02 & recv_frames_[0].data[1]) msg.io_fb_unlock = true;	else msg.io_fb_unlock = false;

					if(0x01 & recv_frames_[0].data[1]) msg.io_fb_lower_beam_headlamp = true;	else msg.io_fb_lower_beam_headlamp = false;
	
					if(0x02 & recv_frames_[0].data[1]) msg.io_fb_upper_beam_headlamp = true;	else msg.io_fb_upper_beam_headlamp = false;

					msg.io_fb_turn_lamp = (0x0c & recv_frames_[0].data[1]) >> 2;

					if(0x10 & recv_frames_[0].data[1]) msg.io_fb_braking_lamp = true;	else msg.io_fb_braking_lamp = false;

					if(0x20 & recv_frames_[0].data[1]) msg.io_fb_clearance_lamp = true;	else msg.io_fb_clearance_lamp = false;

					if(0x40 & recv_frames_[0].data[1]) msg.io_fb_fog_lamp = true;	else msg.io_fb_fog_lamp = false;

					if(0x01 & recv_frames_[0].data[2]) msg.io_fb_speaker = true;	else msg.io_fb_speaker = false;

					if(0x01 & recv_frames_[0].data[3]) msg.io_fb_fl_impact_sensor = true;	else msg.io_fb_fl_impact_sensor = false;

					if(0x02 & recv_frames_[0].data[3]) msg.io_fb_fm_impact_sensor = true;	else msg.io_fb_fm_impact_sensor = false;

					if(0x04 & recv_frames_[0].data[3]) msg.io_fb_fr_impact_sensor = true;	else msg.io_fb_fr_impact_sensor = false;

					if(0x08 & recv_frames_[0].data[3]) msg.io_fb_rl_impact_sensor = true;	else msg.io_fb_rl_impact_sensor = false;

					if(0x10 & recv_frames_[0].data[3]) msg.io_fb_rm_impact_sensor = true;	else msg.io_fb_rm_impact_sensor = false;

					if(0x20 & recv_frames_[0].data[3]) msg.io_fb_rr_impact_sensor = true;	else msg.io_fb_rr_impact_sensor = false;

					if(0x01 & recv_frames_[0].data[4]) msg.io_fb_fl_drop_sensor = true;	else msg.io_fb_fl_drop_sensor = false;

					if(0x02 & recv_frames_[0].data[4]) msg.io_fb_fm_drop_sensor = true;	else msg.io_fb_fm_drop_sensor = false;

					if(0x04 & recv_frames_[0].data[4]) msg.io_fb_fr_drop_sensor = true;	else msg.io_fb_fr_drop_sensor = false;

					if(0x08 & recv_frames_[0].data[4]) msg.io_fb_rl_drop_sensor = true;	else msg.io_fb_rl_drop_sensor = false;

					if(0x10 & recv_frames_[0].data[4]) msg.io_fb_rm_drop_sensor = true;	else msg.io_fb_rm_drop_sensor = false;

					if(0x20 & recv_frames_[0].data[4]) msg.io_fb_rr_drop_sensor = true;	else msg.io_fb_rr_drop_sensor = false;

					if(0x01 & recv_frames_[0].data[5]) msg.io_fb_estop = true;	else msg.io_fb_estop = false;

					if(0x02 & recv_frames_[0].data[5]) msg.io_fb_joypad_ctrl = true;	else msg.io_fb_joypad_ctrl = false;

					if(0x04 & recv_frames_[0].data[5]) msg.io_fb_charge_state = true;	else msg.io_fb_charge_state = false;

					unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

					if(crc == recv_frames_[0].data[7])
					{
							
						io_fb_pub_->publish(msg);
					}

					break;
				}

				case 0x98C4DCEF:
				{
					piot_can_msgs::msg::FrontAngleFb msg;
					msg.front_angle_fb_l = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

					msg.front_angle_fb_r = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

					unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

					if(crc == recv_frames_[0].data[7])
					{
								
						front_angle_fb_pub_->publish(msg);
					}

					break;
				}

				case 0x98C4DDEF:
				{
					piot_can_msgs::msg::RearAngleFb msg;
					msg.rear_angle_fb_l = (float)((short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

					msg.rear_angle_fb_r = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

					unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

					if(crc == recv_frames_[0].data[7])
					{
								
						rear_angle_fb_pub_->publish(msg);
					}

					break;
				}

				case 0x98C4E1EF:
				{
					piot_can_msgs::msg::BmsFb msg;
					msg.bms_fb_voltage = (float)((unsigned short)(recv_frames_[0].data[1] << 8 | recv_frames_[0].data[0])) / 100;

					msg.bms_fb_current = (float)((short)(recv_frames_[0].data[3] << 8 | recv_frames_[0].data[2])) / 100;

					msg.bms_fb_remaining_capacity = (float)((unsigned short)(recv_frames_[0].data[5] << 8 | recv_frames_[0].data[4])) / 100;

					unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

					if(crc == recv_frames_[0].data[7])
					{
								
						bms_fb_pub_->publish(msg);
					}

					break;
				}

				case 0x98C4E2EF:
				{
					piot_can_msgs::msg::BmsFlagFb msg;
					msg.bms_flag_fb_soc = recv_frames_[0].data[0];

					if(0x01 & recv_frames_[0].data[1]) msg.bms_flag_fb_single_ov = true;	else msg.bms_flag_fb_single_ov = false;

					if(0x02 & recv_frames_[0].data[1]) msg.bms_flag_fb_single_uv = true;	else msg.bms_flag_fb_single_uv = false;

					if(0x04 & recv_frames_[0].data[1]) msg.bms_flag_fb_ov = true;	else msg.bms_flag_fb_ov = false;

					if(0x08 & recv_frames_[0].data[1]) msg.bms_flag_fb_uv = true;	else msg.bms_flag_fb_uv = false;

					if(0x10 & recv_frames_[0].data[1]) msg.bms_flag_fb_charge_ot = true;	else msg.bms_flag_fb_charge_ot = false;

					if(0x20 & recv_frames_[0].data[1]) msg.bms_flag_fb_charge_ut = true;	else msg.bms_flag_fb_charge_ut = false;

					if(0x40 & recv_frames_[0].data[1]) msg.bms_flag_fb_discharge_ot = true;	else msg.bms_flag_fb_discharge_ot = false;

					if(0x80 & recv_frames_[0].data[1]) msg.bms_flag_fb_discharge_ut = true;	else msg.bms_flag_fb_discharge_ut = false;

					if(0x01 & recv_frames_[0].data[2]) msg.bms_flag_fb_charge_oc = true;	else msg.bms_flag_fb_charge_oc = false;

					if(0x02 & recv_frames_[0].data[2]) msg.bms_flag_fb_discharge_oc = true;	else msg.bms_flag_fb_discharge_oc = false;

					if(0x04 & recv_frames_[0].data[2]) msg.bms_flag_fb_short = true;	else msg.bms_flag_fb_short = false;

					if(0x08 & recv_frames_[0].data[2]) msg.bms_flag_fb_ic_error = true;	else msg.bms_flag_fb_ic_error = false;

					if(0x10 & recv_frames_[0].data[2]) msg.bms_flag_fb_lock_mos = true;	else msg.bms_flag_fb_lock_mos = false;

					if(0x20 & recv_frames_[0].data[2]) msg.bms_flag_fb_charge_flag = true;	else msg.bms_flag_fb_charge_flag = false;

					msg.bms_flag_fb_hight_temperature = (float)((short)(recv_frames_[0].data[4] << 4 | recv_frames_[0].data[3] >> 4)) / 10;

					msg.bms_flag_fb_low_temperature = (float)((short)((recv_frames_[0].data[6] & 0x0f) << 8 | recv_frames_[0].data[5])) / 10;

					unsigned char crc = recv_frames_[0].data[0] ^ recv_frames_[0].data[1] ^ recv_frames_[0].data[2] ^ recv_frames_[0].data[3] ^ recv_frames_[0].data[4] ^ recv_frames_[0].data[5] ^ recv_frames_[0].data[6];

					if(crc == recv_frames_[0].data[7])
					{
								
						bms_flag_fb_pub_->publish(msg);
					}

					break;
				}

				default:
					break;
			}
		}					
	}

}

}




int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	
//	auto node = std::make_shared<piot_tool::CanControl>();
	rclcpp::spin(std::make_shared<piot_tool::CanControl>());
	


	return 0;
}
