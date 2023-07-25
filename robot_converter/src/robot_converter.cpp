#include "robot_converter.hpp"

using namespace std::chrono_literals;

namespace robot_converter{


    robot_converter::robot_converter(const rclcpp::NodeOptions & options)
    : Node("robot_converter", options)
    {
        RCLCPP_INFO(get_logger(),"robot_converter starting");
        cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",rclcpp::SystemDefaultsQoS(),
        std::bind(&robot_converter::Cmd_VelReceived,this,std::placeholders::_1));

        io_fb_sub = create_subscription<piot_can_msgs::msg::IoFb>(
            "io_fb",rclcpp::SystemDefaultsQoS(),
        std::bind(&robot_converter::Io_FbReceived,this,std::placeholders::_1));

        imu_sub = create_subscription<sensor_msgs::msg::Imu>(
            "imu_sensing",rclcpp::SystemDefaultsQoS(),
        std::bind(&robot_converter::Imu_Received,this,std::placeholders::_1));

        ctrl_fb_sub =create_subscription<piot_can_msgs::msg::CtrlFb>(
            "ctrl_fb",rclcpp::SystemDefaultsQoS(),
        std::bind(&robot_converter::CtrlFb_Received,this,std::placeholders::_1));

        ctrl_cmd_pub = create_publisher<piot_can_msgs::msg::CtrlCmd>("ctrl_cmd",1);
        odom_pub = create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry",1);
        odom_pub2 = create_publisher<nav_msgs::msg::Odometry>("/imu_wheel/odometry",1);
        timer_ = this->create_wall_timer(20ms,std::bind(&robot_converter::timerCallback,this));

        time_flag =false;
        fb.ctrl_fb_gear = 0.;
        fb.ctrl_fb_linear = 0.;
        fb.ctrl_fb_angular = 0.;
        fb.ctrl_fb_slipangle = 0. ;


    }

    void robot_converter::timerCallback(){
        RCLCPP_INFO(get_logger(),"timer callback");
        time_n = this->now();
        float dt= 0.02;
        // if(time_flag){dt = (time_n.seconds() - time_o.seconds()) + (time_n.nanoseconds()/1e+9 - time_o.nanoseconds()/1e+9);}
        // else{dt =0.0;}

        float dx = (c_robot.v_x * cos (c_robot.th) - c_robot.v_y * sin(c_robot.th)) * dt ;
        float dy = (c_robot.v_x * sin (c_robot.th) - c_robot.v_y * cos(c_robot.th)) * dt ;

        float euler[3];
        robot_converter::euler_from_quaternion(current_imu.orientation.x, current_imu.orientation.y , current_imu.orientation.z, current_imu.orientation.w, euler);
        c_robot2.th = euler[2];

        float dx2 = (c_robot2.v_x * cos (c_robot2.th) - c_robot2.v_y * sin(c_robot2.th)) * dt ;
        float dy2= (c_robot2.v_x * sin (c_robot2.th) - c_robot2.v_y * cos(c_robot2.th)) * dt ;
        float dth = c_robot.v_th * dt ;
        c_robot.x += dx;
        c_robot.y += dy;
        c_robot2.x += dx2;
        c_robot2.y += dy2;
        c_robot.th += dth;

        robot_converter::odomfix();

        // time_o = time_n;
        // if(!time_flag){time_flag =true;}
    }

    void robot_converter::odomfix(){


        float quaternion[4];
        robot_converter::quaternion_from_euler(0,0,c_robot.th ,quaternion);
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = c_robot.x;
        odom.pose.pose.position.y = c_robot.y;
        odom.pose.pose.position.z = 0.0;

        odom.twist.twist.linear.x = c_robot.v_x;
        odom.twist.twist.linear.y = c_robot.v_y;
        odom.twist.twist.angular.z = c_robot.v_th;

        odom.pose.pose.orientation.x = quaternion[0];
        odom.pose.pose.orientation.y = quaternion[1];
        odom.pose.pose.orientation.z = quaternion[2];
        odom.pose.pose.orientation.w = quaternion[3];
        for(int i=0; i<36; i++)
        {odom.pose.covariance[i] = pose_covariance[i];}
        for(int i=0; i<36; i++)
        {odom.twist.covariance[i] =twist_covariance[i];}

        odom_pub->publish(odom);

        #ifdef imu_wheel

        odom2.header.stamp = this->now();
        odom2.header.frame_id = "odom";
        odom2.child_frame_id = "base_link2";

        odom2.pose.pose.position.x = c_robot2.x;
        odom2.pose.pose.position.y = c_robot2.y;
        odom2.pose.pose.position.z = 0.0;

        odom2.twist.twist.linear.x = c_robot2.v_x;
        odom2.twist.twist.linear.y = c_robot2.v_y;
        odom2.twist.twist.angular.z = c_robot2.v_th;

        odom2.pose.pose.orientation.x = current_imu.orientation.x;
        odom2.pose.pose.orientation.y = current_imu.orientation.y;
        odom2.pose.pose.orientation.z = current_imu.orientation.z;
        odom2.pose.pose.orientation.w = current_imu.orientation.w;

        for(int i=0; i<36; i++)
        {odom2.pose.covariance[i] = pose_covariance[i];}
        for(int i=0; i<36; i++)
        {odom2.twist.covariance[i] =twist_covariance[i];}
        odom_pub2->publish(odom2);
        #endif

    }
    void robot_converter::Cmd_VelReceived(const geometry_msgs::msg::Twist::SharedPtr msg)
    {


        if (fabs(msg->linear.y) >0 ){
        cmd.ctrl_cmd_gear = 7;
        // RCLCPP_INFO(get_logger(),"msg: x y  %4f , %4f" ,msg->linear.x , msg->linear.y);
        cmd.ctrl_cmd_linear = sqrt(pow(msg->linear.x,2) +pow(msg->linear.y,2));
        
        cmd.ctrl_cmd_angular = 0.0;
        cmd.ctrl_cmd_slipangle = atan2(msg->linear.y , msg->linear.x) * 57.2958 ;
        // if (msg->linear.y >0){cmd.ctrl_cmd_slipangle * -1;}
        // RCLCPP_INFO(get_logger(),"this is slip %4f", cmd.ctrl_cmd_slipangle);
        // if (abs(cmd.ctrl_cmd_slipangle)>90.)
        // {
        //     cmd.ctrl_cmd_linear = cmd.ctrl_cmd_linear * -1 ;
        //     if(cmd.ctrl_cmd_slipangle >0 ){
        //         cmd.ctrl_cmd_slipangle = cmd.ctrl_cmd_slipangle - 180;
        //     }
        //     else {cmd.ctrl_cmd_slipangle = cmd.ctrl_cmd_slipangle + 180;}
        // }
        ctrl_cmd_pub->publish(cmd);

        }
        else{
        cmd.ctrl_cmd_gear = 6;
        cmd.ctrl_cmd_linear = msg->linear.x;
        cmd.ctrl_cmd_angular = msg->angular.z * 57.2958;
        cmd.ctrl_cmd_slipangle = 0.0;
        ctrl_cmd_pub->publish(cmd);
        
        }

    //   ctrl_cmd_pub->publish(cmd);

    }

    void robot_converter::Io_FbReceived(const piot_can_msgs::msg::IoFb::SharedPtr msg)
    {

    }

    void robot_converter::CtrlFb_Received(const piot_can_msgs::msg::CtrlFb::SharedPtr msg)
    {
        fb = *msg;
        if (fb.ctrl_fb_gear ==6 ){
            c_robot.v_x = fb.ctrl_fb_linear ;
            c_robot.v_y = 0.0;
            c_robot.v_th = fb.ctrl_fb_angular * 0.0174533; //deg2rad
            #ifdef imu_wheel
            c_robot2.v_x = fb.ctrl_fb_linear ;
            c_robot2.v_y = 0.0;
            c_robot2.v_th = fb.ctrl_fb_angular * 0.0174533; //deg2rad
            #endif

        }

        else if(fb.ctrl_fb_gear == 7){
            c_robot.v_x = fb.ctrl_fb_linear * cos (fb.ctrl_fb_slipangle * 0.0174533);
            c_robot.v_y = fb.ctrl_fb_linear * sin (fb.ctrl_fb_slipangle * 0.0174533);
            c_robot.v_th = 0.0;}

        else {
            c_robot.v_x = 0.0;
            c_robot.v_y = 0.0;
            c_robot.v_th = 0.0;
            #ifdef imu_wheel
            c_robot2.v_x = fb.ctrl_fb_linear;
            c_robot2.v_y = 0.0;
            c_robot2.v_th = current_imu.angular_velocity.z;
            #endif
        }


    }

    void robot_converter::Imu_Received(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        current_imu = *msg;

    }



}
