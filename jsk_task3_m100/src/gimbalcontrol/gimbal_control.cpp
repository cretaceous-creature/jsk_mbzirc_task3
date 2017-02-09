/*
 * jsk MBZIRC
 */

// Author: author: chen

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
//dji sdk
#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/dji_drone.h>
using namespace DJI::onboardSDK;

class Gimbalcontrol
{
private:
    //data

    //subscriber and service
    ros::NodeHandle n_;
    ros::Subscriber odom_sub_;
    //publisher

    //dji sdk
    DJIDrone* DJI_M100;

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom);
public:
    void init()
    {
        //DJI SDK
        DJI_M100 = new DJIDrone(n_);
        //data init
        DJI_M100->gimbal_angle_control(0,0,-900,20); //2 senconds send pitch to 90 degree
        //sub and srv
        odom_sub_ = n_.subscribe<nav_msgs::Odometry>("/dji_sdk/odometry",10,&Gimbalcontrol::OdomCallback,this);
        //ros::NodeHandle n_private("~");
        //n_private.param("vertical_vel", vertical_vel, 1.0);


    }
    bool teleopUGV;
    ~Gimbalcontrol()   {
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "gimbalcontrol");

    Gimbalcontrol gbc;
    gbc.init();

    puts("Reading from joy");
    
    ros::spin();

    return(0);
}

/***************
 * 50hz, control the Gimbal
 * to face bottom
 * ************/
void Gimbalcontrol::OdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
    tf::Pose pose;
    tf::poseMsgToTF(odom->pose.pose,pose);
    double roll,pitch,yaw;
    tf::Matrix3x3(pose.getRotation()).getEulerYPR(yaw,pitch,roll);
//    ROS_INFO("%f,%f,%f \n", roll,pitch,yaw);
//    //if(pitch<0)
//    //     pitch /= 2.5;
//    //roll *=1.5;
//    roll = 1800*roll/3.14;
//    pitch = 1800*pitch/3.14;
//    yaw = 1800*yaw/3.14;
//    ROS_INFO("%f,%f,%f \n", roll,pitch,yaw);
//    DJI_M100->gimbal_angle_control((int)roll, -(int)pitch - 900, 0, 10);

}
