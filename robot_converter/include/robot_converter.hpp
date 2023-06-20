#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/rclcpp.hpp"
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

#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "message_filters/subscriber.h"
#include <chrono>

#define imu_wheel

struct robot_odom{
    float x;
    float y;
    float th;
    float v_x;
    float v_y;
    float v_th;

};

namespace robot_converter
{

    class robot_converter : public rclcpp::Node{

        public:
            explicit robot_converter (const rclcpp::NodeOptions &);

            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub2;
            rclcpp::Publisher<piot_can_msgs::msg::CtrlCmd>::SharedPtr ctrl_cmd_pub;

            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
            rclcpp::Subscription<piot_can_msgs::msg::CtrlFb>::SharedPtr ctrl_fb_sub;
            rclcpp::Subscription<piot_can_msgs::msg::IoFb>::SharedPtr io_fb_sub;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
            // piot_can_msgs::msg::CtrlCmd 

            rclcpp::TimerBase::SharedPtr timer_;

            void Cmd_VelReceived(const geometry_msgs::msg::Twist::SharedPtr msg);
            void Io_FbReceived(const piot_can_msgs::msg::IoFb::SharedPtr msg);
            void Imu_Received(const sensor_msgs::msg::Imu::SharedPtr msg);
            void CtrlFb_Received(const piot_can_msgs::msg::CtrlFb::SharedPtr msg);
            void timerCallback();
            void odomfix();


            inline void quaternion_from_euler( float raw , float pitch , float yaw, float array[])
            {
                float ai = raw / 2.0;
                float aj = pitch / 2.0;
                float ak = yaw / 2.0;

                float ci = cos(ai);
                float si = sin(ai);
                float cj = cos(aj);
                float sj = sin(aj);
                float ck = cos(ak);
                float sk = sin(ak);
                float cc = ci*ck;
                float cs = ci*sk;
                float sc = si*ck;
                float ss = si*sk;
                array[0] = cj*sc - sj*cs;
                array[1] = cj*ss + sj*cc;
                array[2] = cj*cs - sj*sc;
                array[3] = cj*cc + sj*ss;
            };

            inline void euler_from_quaternion(float x, float y, float z, float w , float array[]){
                float t0 = 2.0 * (w *x + y*z);
                float t1 = 1.0 - 2.0 * (x*x + y*y);
                array[0] = atan2(t0,t1);

                float t2 = 2.0 * (w*z + x*y);
                if(t2> 1.0){ t2 = 1.0; }
                if(t2<-1.0){ t2 = -1.0;}
                array[1] = asin(t2);

                float t3 = 2.0 * (w*z + x*y);
                float t4 = 1.0 - 2.0 * (y*y + z*z);
                array[2] = atan2(t3,t4);
            }

            robot_odom c_robot;
            robot_odom c_robot2;
            sensor_msgs::msg::Imu current_imu;

            nav_msgs::msg::Odometry odom;
            nav_msgs::msg::Odometry odom2;

            rclcpp::Time time_n;
            rclcpp::Time time_o;

            piot_can_msgs::msg::CtrlCmd cmd;
            piot_can_msgs::msg::CtrlFb fb;
            bool time_flag ;
    };





}