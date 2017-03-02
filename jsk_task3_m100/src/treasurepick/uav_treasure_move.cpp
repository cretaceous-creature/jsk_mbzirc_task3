/*
 jsk_mbzirc_task
 */

// Author: Chen

#include <ros/ros.h>

//msg headers.
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
//rqt_reconfig
#include <dynamic_reconfigure/server.h>
#include <jsk_task3_m100/ControlparamConfig.h>
//sys lib
#include <iostream>
#include <string>

//dji sdk
#include <dji_sdk/dji_sdk.h>
#include <dji_sdk/dji_drone.h>

//pcl
#include <ctime>




class uav_move
{

private:

    ros::NodeHandle nh_;
    //subscriber
    ros::Subscriber aim_pose_sub_;
    ros::Subscriber uav_odom_sub_;
    ros::Subscriber obj_sub_local_;
    ros::Subscriber guidance_sub_;

    //data field
    nav_msgs::Odometry uav_odom;
    geometry_msgs::Pose aim_pose;
    geometry_msgs::Pose aim_local_pose;
    geometry_msgs::PoseArray obj_centers_local;
    sensor_msgs::LaserScan ultra_sonic;
    const float obj_plane_ = 0.2;//1 meter high
    //cmd_vel
    geometry_msgs::Twist obj_vel;
    geometry_msgs::Twist vel_cmd_uav;
    geometry_msgs::Twist diff_twist, d_diff_twist, i_diff_twist;
    double uav_h;
    //dynamic reconfigure
    dynamic_reconfigure::Server<jsk_mbzirc_tasks::ControlparamConfig> server;
    dynamic_reconfigure::Server<jsk_mbzirc_tasks::ControlparamConfig>::CallbackType f;
    double Kp,Ki,Kd,MAXSPEED;
    // drone name
    DJIDrone* DJI_M100;

public:

    void DynamicReconfigureCallback(jsk_mbzirc_tasks::ControlparamConfig &config, uint32_t level)
    {
        Kp = config.Kp;
        Ki = config.Ki;
        Kd = config.Kd;
        MAXSPEED = config.maxspeed;
        nh_.setParam("Kp",Kp);
        nh_.setParam("Kd",Kd);
        nh_.setParam("Ki",Ki);
        nh_.setParam("Uav_max_velocity",MAXSPEED);

    }

    void init()
    {
        //DJI SDK
        DJI_M100 = new DJIDrone(nh_);

        //subscriber
        aim_pose_sub_ = nh_.subscribe("/aimpose",1,&uav_move::AimPoseCallback,this);
        uav_odom_sub_ = nh_.subscribe("/dji_sdk/odometry",1,&uav_move::OdomCallback,this);
        obj_sub_local_ = nh_.subscribe("/obj_cluster/centroid_pose_local",1,&uav_move::ObjPoseLocalCallback,this);
        guidance_sub_ = nh_.subscribe("/guidance/ultrasonic", 1, &uav_move::UltraSonicCallback, this);

        /*** dynamic reconfigure ***/
        f = boost::bind(&uav_move::DynamicReconfigureCallback, this, _1,_2);
        server.setCallback(f);
    }

    void UltraSonicCallback(const sensor_msgs::LaserScan data)
    {
        ultra_sonic = data; //update ultra sonic...
    }

    void ObjPoseLocalCallback(const geometry_msgs::PoseArray posearray)
    {
        obj_centers_local = posearray; //update the data
    }

    void AimPoseCallback(const geometry_msgs::Pose aimpose)
    {
        for(int i = 0; i < obj_centers_local.poses.size(); i++)
        {

            if(obj_centers_local.poses.at(i).orientation.x == aimpose.orientation.x &&
                    obj_centers_local.poses.at(i).orientation.y == aimpose.orientation.y)
            {
                obj_vel.linear.x = aimpose.position.x - aim_pose.position.x;
                obj_vel.linear.y = aimpose.position.y - aim_pose.position.y;
                obj_vel.linear.z = aimpose.position.z - aim_pose.position.z; //z is not necessary for following
                //obj_vel.linear.x*=2;obj_vel.linear.y*=2;
                aim_pose = aimpose; //renew the aimpose..
                //get the twist
                geometry_msgs::Twist diff;

                //delta diff twist... change of the
                d_diff_twist.linear.x = obj_centers_local.poses.at(i).position.x -
                        aim_local_pose.position.x;
                d_diff_twist.linear.y = obj_centers_local.poses.at(i).position.y -
                        aim_local_pose.position.y;
                d_diff_twist.linear.z = obj_centers_local.poses.at(i).position.z -
                        aim_local_pose.position.z;

                aim_local_pose = obj_centers_local.poses.at(i); // local pose, in drone coordinate
                //intergrate twist
                i_diff_twist.linear.x += aim_local_pose.position.x/1000;
                i_diff_twist.linear.y += aim_local_pose.position.y/1000;
                i_diff_twist.linear.z += aim_local_pose.position.z/1000;

                i_diff_twist.linear.x = i_diff_twist.linear.x>10.0?10.0:i_diff_twist.linear.x;
                i_diff_twist.linear.y = i_diff_twist.linear.y>10.0?10.0:i_diff_twist.linear.y;
                i_diff_twist.linear.z = i_diff_twist.linear.z>5.0?5.0:i_diff_twist.linear.z;
                i_diff_twist.linear.x = i_diff_twist.linear.x<-10.0?-10.0:i_diff_twist.linear.x;
                i_diff_twist.linear.y = i_diff_twist.linear.y<-10.0?-10.0:i_diff_twist.linear.y;
                i_diff_twist.linear.z = i_diff_twist.linear.z<-5.0?-5.0:i_diff_twist.linear.z;
            }
        }
    }


    void OdomCallback(const nav_msgs::OdometryConstPtr odom)
    {
        double uav_h;
        if(odom->pose.pose.position.z<1&&ultra_sonic.ranges.size()&&ultra_sonic.ranges.at(0)>0.05) //less than 1 meter
            uav_h = ultra_sonic.ranges.at(0);
        else
            uav_h = odom->pose.pose.position.z;

        uav_odom = *odom;
        //PID control the diff
       // vel_world_uav = obj_vel;  //send the local velocity of object


        vel_cmd_uav.linear.x = Kp * aim_local_pose.position.x - Kd * d_diff_twist.linear.x;
	//   + Ki*2 * i_diff_twist.linear.x;
        vel_cmd_uav.linear.y = Kp * aim_local_pose.position.y - Kd * d_diff_twist.linear.y;
	//   + Ki*2 * i_diff_twist.linear.y;
        vel_cmd_uav.linear.z = Kp * aim_local_pose.position.z- Kd * d_diff_twist.linear.z;
	//    + Ki * i_diff_twist.linear.z;

        //el_world_uav.linear.z = 0;

        vel_cmd_uav.linear.x = vel_cmd_uav.linear.x>MAXSPEED?MAXSPEED:vel_cmd_uav.linear.x;
        vel_cmd_uav.linear.y = vel_cmd_uav.linear.y>MAXSPEED?MAXSPEED:vel_cmd_uav.linear.y;
        vel_cmd_uav.linear.z = vel_cmd_uav.linear.z>3.0?3.0:vel_cmd_uav.linear.z;

        vel_cmd_uav.linear.x = vel_cmd_uav.linear.x<-MAXSPEED?-MAXSPEED:vel_cmd_uav.linear.x;
        vel_cmd_uav.linear.y = vel_cmd_uav.linear.y<-MAXSPEED?-MAXSPEED:vel_cmd_uav.linear.y;
        vel_cmd_uav.linear.z = vel_cmd_uav.linear.z<-3.0?-3.0:vel_cmd_uav.linear.z;
	double z_velo; 
        if(uav_h > 1.3)
	  z_velo = -0.5;
	else if(uav_h >1.1)
	  z_velo = -0.1;
	else
	  z_velo= 0;
	        DJI_M100->velocity_control(0,vel_cmd_uav.linear.x,-vel_cmd_uav.linear.y,z_velo,0);
        std::cout<<"the velocity is :"<< vel_cmd_uav.linear.x << "," << -vel_cmd_uav.linear.y << ","<<
                   z_velo << std::endl;

    }

    ~uav_move()
    {    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_move");
    uav_move t_p;
    t_p.init();

    ros::spin();

}
