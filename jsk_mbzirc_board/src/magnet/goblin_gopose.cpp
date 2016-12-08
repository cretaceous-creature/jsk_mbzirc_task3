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
#include <geometry_msgs/Pose2D.h>

//service
#include <hrp2g_wheelbase/Gopose.h>


class Gopose
{
private:
    hrp2g_wheelbase::Gopose::Request tmp_req;
    geometry_msgs::Pose2D odom;
    geometry_msgs::Twist cmd_vel;
    ros::NodeHandle nh_;
    ros::Publisher idle_pub_;  // gopose node in idle state
    ros::Publisher cmd_pub_; //publish the /cmd_vel to the base...
    ros::Subscriber odom_sub_;
    ros::ServiceServer gopose_srv_;
    float odom_interval, v_linear_max, v_angular_max;
    int odom_counter = 0;
  float Kp,Kd,Kp_param, Kd_param;
public:
    void init()
    {
        //global parameters
        odom_interval = 0.012048;   //12ms.... 83.33hz for odometry/raw message...
        v_linear_max = 1.5; // m/s
        v_angular_max = 0.8; // rad/s
        Kp_param = 5;
        Kd_param = 0.2;
        nh_.setParam("/goblin/ODOM_TIME_INTERVAL",odom_interval);
        nh_.setParam("/goblin/MAX_LINEAR_V",v_linear_max);
        nh_.setParam("/goblin/MAX_ANGULAR_V",v_angular_max);
        nh_.setParam("/goblin/Kp",Kp_param);
        nh_.setParam("/goblin/Kd",Kd_param);

        // initial the odometry
        odom.x = odom.y = odom.theta = 0;
        //idle publisher publish achieved odom if no gopose orders
        idle_pub_ = nh_.advertise<geometry_msgs::Pose2D>("gopose_results", 1);
        //cmd publisher
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
        //odometry subscriber
        odom_sub_ = nh_.subscribe<geometry_msgs::Twist>("/odometry/raw",1,&Gopose::OdomCallBack,this);
        //service
        gopose_srv_ = nh_.advertiseService("/goblin/gopose", &Gopose::Gopose_srv,this);

    }
    //twist callback
    void OdomCallBack(const geometry_msgs::TwistConstPtr& twist);
    //service callback
    bool Gopose_srv(hrp2g_wheelbase::Gopose::Request &req,
                    hrp2g_wheelbase::Gopose::Response &res);

  ~Gopose()   {exit(0); }
};
// service, input x,theta,time, output status
bool Gopose::Gopose_srv(hrp2g_wheelbase::Gopose::Request &req,
                hrp2g_wheelbase::Gopose::Response &res)
{
    if((req.x/req.time > v_linear_max) || (req.theta/req.time > v_angular_max))
    {
        res.status = 0;
    }
        else
    {
        res.status = 1;
        tmp_req = req;
        odom.x = odom.y = odom.theta = 0; //clear the odom...
        nh_.getParam("/goblin/Kp",Kp_param);
        nh_.getParam("/goblin/Kd",Kd_param);
        Kp = Kp_param/req.time;
        Kd = Kd_param/(req.time*odom_interval);
    }
}
//odometry callback  get the temporary twist...
void Gopose::OdomCallBack(const geometry_msgs::TwistConstPtr& twist)
{
    if(tmp_req.time > 0)
    {
        float d_err_l = 0;  //easy filter, ignore noise less than 1mm/s
        float d_err_a = 0;  //easy filter, ignore noise less than 0.01 rad/s
        if(fabs(twist->linear.x+twist->linear.y)/2>0.0001)
            d_err_l = (twist->linear.x + twist->linear.y)/2;
        if(fabs(twist->angular.z)>0.01)
            d_err_a = twist->angular.z;

        odom.x += d_err_l*odom_interval;
        odom.theta += d_err_a*odom_interval;
        if(odom_counter > 5)
        {
            odom_counter = 0;
            float err_l = tmp_req.x - odom.x;
            float err_a = tmp_req.theta - odom.theta;
            float v_l = Kp*err_l - Kd*d_err_l;
            float v_a = Kp*err_a - Kd*d_err_a;
            v_l = v_l==0? v_l+0.01:v_l;
            cmd_vel.linear.x = v_l;
            cmd_vel.angular.z = v_a;
            cmd_pub_.publish(cmd_vel);
        }
        tmp_req.time -= odom_interval;
    }
    else
    {
        //time over, finished
        if(odom_counter > 5)
        {
            idle_pub_.publish(odom);
            odom_counter = 0;
            if(tmp_req.time<0)
              {
                tmp_req.time = 0;
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                cmd_pub_.publish(cmd_vel);
              }
        }
    }
    odom_counter++;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "hrp2g_gopose");

    Gopose goblin;
    goblin.init();
    ros::spin();

    return(0);
}

