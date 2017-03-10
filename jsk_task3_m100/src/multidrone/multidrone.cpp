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
#include <dji_sdk/GlobalPosition.h>
#include <nav_msgs/Odometry.h>


class Multidrone
{
private:

#define C_EARTH (double) 6378137.0  //using the WGS84 coordinate
#define PI 3.14159265359
    //origin latitude and longitude
    double origin_lati;
    double origin_longi;
    double drone_num;
    //msgs to publish
    nav_msgs::Odometry share_odom;
    ros::Publisher share_odom_pub_;
    //subscriber and service
    ros::NodeHandle *n_;
    ros::Subscriber gps_sub_;
    //publisher
public:
    void init()
    {
        double lati_rc = 24.643803;
        double longi_rc = 54.7564013;

        n_ = new ros::NodeHandle("~");
        n_->param("origin_latitude",  origin_lati, 24.643803 * PI/180);
        n_->param("origin_longitude",  origin_longi, 54.7564013 * PI/180);
        n_->param("drone_number",  drone_num, 1.0);  //1 for m100,
        gps_sub_ = n_->subscribe<dji_sdk::GlobalPosition>("/dji_sdk/global_position", 1 ,&Multidrone::GpsCallback,this);

        //publisher
        std::string topic = n_->resolveName("/share_odometry");
        share_odom_pub_ = n_->advertise<nav_msgs::Odometry>(topic,1);

    }

    void GpsCallback(dji_sdk::GlobalPosition gps_data)
    {
        //check the latitude and longitude to be in the arena

        if(gps_data.latitude > 20 && gps_data.latitude < 30
                && gps_data.longitude > 50 && gps_data.longitude < 60)
        {
            double lati = gps_data.latitude * PI/180;
            double longi = gps_data.longitude * PI/180;
            /* From GPS to Ground */
            double dlati = lati-origin_lati;
            double dlongti= longi-origin_longi;
            double x = dlati * C_EARTH;
            double y = dlongti * C_EARTH * cos(lati / 2.0 + origin_lati / 2.0);

            share_odom.header = gps_data.header;
            share_odom.pose.pose.position.x = x;
            share_odom.pose.pose.position.y = y;
            share_odom.pose.pose.position.z = gps_data.height;
            share_odom.pose.pose.orientation.x = drone_num;

            std::cout << x << "," << y << std::endl;
            share_odom_pub_.publish(share_odom);
        }
        else //with error.... dont know what to do...
        {

        }
    }
    ~Multidrone()   {

    }
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "multidrone");

    Multidrone mtd;
    mtd.init();
    ros::spin();

    return(0);
}
