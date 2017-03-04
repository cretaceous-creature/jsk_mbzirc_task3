/*
 * jsk MBZIRC
 */

// Author: author: chen

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
//dji sdk
#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/dji_drone.h>
using namespace DJI::onboardSDK;

//task3 magnet srv
#include <jsk_mbzirc_board/Magnet.h>

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

class TeleopUAVJoy
{
private:
    //data
    double walk_vel, run_vel, yaw_rate, yaw_rate_run, vertical_vel;
    u_int32_t v_rc[16],v_rc_checker, stampsec, stampcounter;  //virtual RC
    geometry_msgs::Twist cmd;
    jsk_mbzirc_board::Magnet mag_status_;
    //subscriber and service
    ros::NodeHandle n_;
    ros::Subscriber joy_sub_;
    ros::ServiceClient mag_srv_;
    //publisher

    //dji sdk
    DJIDrone* DJI_M100;

    void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
public:
    void init()
    {
        //DJI SDK
        DJI_M100 = new DJIDrone(n_);
        //data init
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
        stampcounter =0;
	v_rc_checker = 0;
        v_rc[0] = 1024; // roll    +- 660
        v_rc[1] = 1024; // pitch
        v_rc[2] = 1024; // throttle
        v_rc[3] = 1024; // yaw
        v_rc[4] = 1324; // gear
        v_rc[6] = 496;  // mode {1552(P), 1024(A), 496(F)}
        //sub and srv
        joy_sub_ = n_.subscribe<sensor_msgs::Joy>("/from_fc/joy",10,&TeleopUAVJoy::JoyCallback,this);
        mag_srv_ = n_.serviceClient<jsk_mbzirc_board::Magnet>("serial_board/magnet_control");
        ros::NodeHandle n_private("~");
        n_private.param("walk_vel", walk_vel, 1.0);
        n_private.param("run_vel", run_vel, 4.0);
        n_private.param("yaw_rate", yaw_rate, 0.5);
        n_private.param("yaw_run_rate", yaw_rate_run, 1.0);
        n_private.param("vertical_vel", vertical_vel, 1.0);


    }
    bool teleopUGV;
    ~TeleopUAVJoy()   {
        DJI_M100->gohome();
    }
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_teleop_joy");

    TeleopUAVJoy tpj;
    tpj.init();

    signal(SIGINT,quit);
    puts("Reading from joy");
    puts("---------------------------");
    puts("Hold 'L1' for nomal movement");
    puts("Use 'left axis' to horizontal translate");
    puts("Use 'right axis' to yaw and up/down");
    puts("Hold 'R1' for fast movement");
      puts("for test333");
    ros::spin();

    return(0);
}

void TeleopUAVJoy::JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    bool dirty=false;
    bool dirtygripper=false;

    cmd.linear.x = cmd.linear.y = cmd.angular.z = cmd.linear.z = 0;
    //if the joy get last, check the second, too large then stop...
    if(stampsec==joy->header.stamp.sec)
        stampcounter++;
    else
        stampcounter = 0;

    stampsec = joy->header.stamp.sec;
    if(stampsec==0)
      {ROS_INFO("bad joy data");return; }
    if(stampcounter > 20)
      {DJI_M100->velocity_control(0,0,0,0,0);ROS_INFO("joy data lost, wait for recon");}
    //Gripper
    if(joy->buttons[PS3_AXIS_BUTTON_ACTION_CIRCLE]&&joy->buttons[PS3_AXIS_BUTTON_ACTION_TRIANGLE])
    {
        //release the gripper
        mag_status_.request.on = false;
        mag_status_.request.time_ms = 1000;
        if(mag_srv_.call(mag_status_))
            ROS_INFO("Gripper Released for 1000 ms");
        else
            ROS_INFO("Fail to release the gripper");
        ros::Duration(0.2).sleep();
        dirtygripper = true;
    }
    if(joy->buttons[PS3_AXIS_BUTTON_ACTION_SQUARE]&&joy->buttons[PS3_AXIS_BUTTON_ACTION_TRIANGLE])
    {
        //release the gripper
        mag_status_.request.on = false;
        mag_status_.request.time_ms = 0;
        if(mag_srv_.call(mag_status_))
            ROS_INFO("Gripper Disable");
        else
            ROS_INFO("Fail to disable gripper");
        ros::Duration(0.2).sleep();
        dirtygripper = true;
    }

    //Obtain and release control ability
    {
        if(joy->buttons[PS3_BUTTON_REAR_LEFT_2]&&joy->buttons[PS3_BUTTON_REAR_RIGHT_2]&&
                joy->buttons[PS3_AXIS_BUTTON_ACTION_CROSS])
        {
            DJI_M100->request_sdk_permission_control();
            ros::Duration(0.2).sleep();
        }
        if(joy->buttons[PS3_BUTTON_REAR_LEFT_2]&&joy->buttons[PS3_BUTTON_REAR_RIGHT_2]&&
                joy->buttons[PS3_AXIS_BUTTON_ACTION_CIRCLE])
        {
            DJI_M100->release_sdk_permission_control();
            ros::Duration(0.2).sleep();
        }
    }
    //Take off and Landing
    {
        if(joy->buttons[PS3_BUTTON_REAR_LEFT_1]&&joy->buttons[PS3_BUTTON_REAR_RIGHT_1]&&
                joy->buttons[PS3_AXIS_BUTTON_ACTION_CROSS])
           { DJI_M100->takeoff();        ros::Duration(0.2).sleep();}
        if(joy->buttons[PS3_BUTTON_REAR_LEFT_1]&&joy->buttons[PS3_BUTTON_REAR_RIGHT_1]&&
                joy->buttons[PS3_AXIS_BUTTON_ACTION_CIRCLE])
           { DJI_M100->landing();        ros::Duration(0.2).sleep();}
        if(joy->buttons[PS3_BUTTON_REAR_LEFT_1]&&joy->buttons[PS3_BUTTON_REAR_RIGHT_1]&&
                joy->buttons[PS3_AXIS_BUTTON_ACTION_SQUARE])
           { DJI_M100->gohome();        ros::Duration(0.2).sleep();}
    }
    //Gimbal
    if(joy->buttons[PS3_BUTTON_REAR_LEFT_2])
    {
        if(joy->buttons[PS3_AXIS_BUTTON_ACTION_CROSS])
        {
            DJI_M100->gimbal_angle_control(0,-900,0,10);
        }
        else
        {
            float pitch = 300*joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
            float roll = -150*joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
            float yaw = -300*joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
            DJI_M100->gimbal_speed_control(roll,pitch,yaw);
        }
    }

    //Velocity
    if(joy->buttons[PS3_BUTTON_REAR_LEFT_1])
    {
        float max_vel = walk_vel;
        float max_yawrate = yaw_rate;
        if(joy->buttons[PS3_BUTTON_REAR_RIGHT_1])
        {
            max_vel = run_vel;
            max_yawrate = yaw_rate_run;
        }
        cmd.linear.x = max_vel*joy->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
        cmd.linear.y = -(max_vel*joy->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS]);
        cmd.linear.z = vertical_vel*joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
        cmd.angular.z = -100*max_yawrate*joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
        DJI_M100->velocity_control(0,cmd.linear.x,cmd.linear.y,cmd.linear.z,cmd.angular.z);
	// ROS_INFO("linear and angular z is %f, %f",cmd.linear.z,cmd.angular.z);
    }

    //Virtual_RC
    if(joy->buttons[PS3_BUTTON_START])
    {
        v_rc_checker++;
        ros::Duration(0.1).sleep();
	if(v_rc_checker%2)
	  {
	    ROS_INFO("ENABLE VIRTUAL RC MODE, BE CAREFUL!");
	  }
	else
	  {
	    ROS_INFO("QUIT VIRTUAL RC MODE, PLEASE USE DJI RC");
	  }
    }
    if(v_rc_checker%2)
    {
        DJI_M100->virtual_rc_enable();
        DJI_M100->virtual_rc_control(v_rc);
    }
    else
    {
        DJI_M100->virtual_rc_disable();
    }
}
