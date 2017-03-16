/*
 jsk_mbzirc_task
 */

// Author: Chen
 

#include <ros/ros.h>

//msg headers.
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
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
    ros::Subscriber lidar_sub_;
    float lidar_data;
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
    unsigned int control_counter = 0;

    //dirty counter
    int lostcounter = 0;
    double last_vx = 0;
    double last_vy = 0;
    int drone_num = 0;
    // the drone's height could be less than 1 meter for only 10 seconds..
    int holdupcounter = 0;

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
        lidar_sub_ = nh_.subscribe<std_msgs::Float32>("/lidar_laser",1,&uav_move::LidarCallback,this);
        /*** dynamic reconfigure ***/
        f = boost::bind(&uav_move::DynamicReconfigureCallback, this, _1,_2);
        server.setCallback(f);
    }
    void LidarCallback(const std_msgs::Float32 data)
    {
        lidar_data = (float)data.data;
        lidar_data /= 100; //from cm to meter
    }
    void UltraSonicCallback(const sensor_msgs::LaserScan data)
    {
        drone_num = 1;
        if(data.ranges.size())
            if(data.ranges.at(0)<0.37 || data.ranges.at(0)>0.41)
                ultra_sonic = data; //update ultra sonic...
    }

    void ObjPoseLocalCallback(const geometry_msgs::PoseArray posearray)
    {
        obj_centers_local = posearray; //update the data
    }

    void AimPoseCallback(const geometry_msgs::Pose aimpose)
    {
      aim_pose = aimpose;
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
        if(odom->pose.pose.position.z<2.0&&ultra_sonic.ranges.size()&&ultra_sonic.ranges.at(0)>0.05) //less than 1 meter
            uav_h = ultra_sonic.ranges.at(0);
        else if(odom->pose.pose.position.z<3&&lidar_data>0&&lidar_data<3) //less than 3 meters
            uav_h = lidar_data;
        else
        {
            if(drone_num == 1)
               uav_h = odom->pose.pose.position.z + 0.1;
            else
               uav_h = odom->pose.pose.position.z + 0.18;

        }
	//only use gps
	//	uav_h = odom->pose.pose.position.z;

        uav_odom = *odom;
        if(aim_pose.orientation.w == 1.0)
        {
            // searching status
            std::cout<<"the drone is in searching status.."<<std::endl;
            double unify = sqrt(aim_pose.position.x*aim_pose.position.x +
                                aim_pose.position.y*aim_pose.position.y);
            vel_cmd_uav.linear.x = 1.3 * (aim_pose.position.x / unify);
            vel_cmd_uav.linear.y = 1.3 * (aim_pose.position.y / unify);
            vel_cmd_uav.linear.z = aim_pose.position.z - uav_h;

            vel_cmd_uav.linear.z = vel_cmd_uav.linear.z>0.3?0.3:vel_cmd_uav.linear.z;
            vel_cmd_uav.linear.z = vel_cmd_uav.linear.z<-0.3?-0.3:vel_cmd_uav.linear.z;

            //control in global frame....
            DJI_M100->velocity_control(1,vel_cmd_uav.linear.x,vel_cmd_uav.linear.y,vel_cmd_uav.linear.z,0);
            //we can comment this and check...
            std::cout<<"I am searching, and the velocity is: "<< vel_cmd_uav.linear.x <<" , "
                    << vel_cmd_uav.linear.y <<" , " <<
                       vel_cmd_uav.linear.z << std::endl;
            control_counter = 0;

        }
        else
        {
            // let Kp be bigger when the offset is small
            double Kp_t;
            Kp_t = Kp*0.8;
            if(uav_h<1.5)
                Kp_t *= 1.5;   //1 - 2.5 times....


            vel_cmd_uav.linear.x = Kp_t * aim_local_pose.position.x - Kd * d_diff_twist.linear.x
               + Ki * i_diff_twist.linear.x;
            vel_cmd_uav.linear.y = Kp_t * aim_local_pose.position.y - Kd * d_diff_twist.linear.y
               + Ki * i_diff_twist.linear.y;
            vel_cmd_uav.linear.z = Kp_t * aim_local_pose.position.z- Kd * d_diff_twist.linear.z;
            //    + Ki * i_diff_twist.linear.z;

            //el_world_uav.linear.z = 0;

            vel_cmd_uav.linear.x = vel_cmd_uav.linear.x>MAXSPEED?MAXSPEED:vel_cmd_uav.linear.x;
            vel_cmd_uav.linear.y = vel_cmd_uav.linear.y>MAXSPEED?MAXSPEED:vel_cmd_uav.linear.y;
            vel_cmd_uav.linear.z = vel_cmd_uav.linear.z>3.0?3.0:vel_cmd_uav.linear.z;

            vel_cmd_uav.linear.x = vel_cmd_uav.linear.x<-MAXSPEED?-MAXSPEED:vel_cmd_uav.linear.x;
            vel_cmd_uav.linear.y = vel_cmd_uav.linear.y<-MAXSPEED?-MAXSPEED:vel_cmd_uav.linear.y;
            vel_cmd_uav.linear.z = vel_cmd_uav.linear.z<-3.0?-3.0:vel_cmd_uav.linear.z;
            double z_velo;
            if(uav_h > 1.2)
                z_velo = -0.35;
            else if(uav_h >1.1)
                z_velo = -0.1;
            else
                z_velo= 0;

            if(aim_pose.orientation.w == 2)
            {
                // now making attemp....
                if(fabs(aim_local_pose.position.x)<0.1 && fabs(aim_local_pose.position.y)<0.1)
                {
                    //vel_cmd_uav.linear.x = 0;
                    //vel_cmd_uav.linear.y = 0;
                    z_velo = - 0.999;
                }
                std::cout<<"I am picking..."<<std::endl;

            }
            else if(aim_pose.orientation.w == 3)
            {
                // go back to 3 meters high
                //                DJI_M100->local_position_control(aim_pose.position.x,aim_pose.position.y,3,0);
                if(uav_h<3.0)
                    DJI_M100->velocity_control(0,0,0,0.5,0);
                else
                    DJI_M100->velocity_control(0,0,0,0,0);
                // and we should disable velocity control
                control_counter = 200; //  300 equals to 6 seconds....
                std::cout<<"I am holding up....."<<std::endl;
            }


            // set a counter to always minus the frequency should be 50 HZ.....
            if(control_counter)
            {
                if(uav_h<3.0)
                    DJI_M100->velocity_control(0,0,0,0.5,0);
                else
                    DJI_M100->velocity_control(0,0,0,0,0);
                control_counter--; //minus ultil 0
            }
            else
            {

                //check if the velocity remains...which means losted...
                if(last_vx == vel_cmd_uav.linear.x && last_vy == vel_cmd_uav.linear.y
                        && last_vy!=0 &&last_vx!=0
                        && last_vy <1 && last_vx<1)
                {
                    lostcounter++;
                    if(lostcounter > 300) // lost for 4 seconds..
                    {
                       std::cout << "i am losting... holding up" << std::endl;

                        if(uav_h<3.0)
                            DJI_M100->velocity_control(0,0,0,1,0);
                        else
                        {
                            DJI_M100->velocity_control(0,0,0,0,0);
                            lostcounter = 0;
                        }
                    }
                }
                else
                {
                    DJI_M100->velocity_control(0,vel_cmd_uav.linear.x,-vel_cmd_uav.linear.y,z_velo,0);
                }
                last_vx = vel_cmd_uav.linear.x;
                last_vy = vel_cmd_uav.linear.y;
                std::cout<<"the velocity is :"<< vel_cmd_uav.linear.x << "," << -vel_cmd_uav.linear.y << ","<<
                           z_velo << std::endl;
            }
        }
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
