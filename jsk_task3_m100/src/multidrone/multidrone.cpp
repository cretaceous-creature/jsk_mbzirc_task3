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


class Multidrone
{
private:

#define C_EARTH (double) 6378137.0  //using the WGS84 coordinate
#define PI 3.14159265359
    //origin latitude and longitude
    double origin_lati;
    double origin_longi;
    //subscriber and service
    ros::NodeHandle *n_;
    ros::Subscriber gps_sub_;
    //publisher
public:
    void init()
    {
        origin_lati = 35.89396184 * PI/180;
        origin_longi = 139.944021504 * PI/180;
        n_ = new ros::NodeHandle("~");
        n_->getParam("origin_latitude", origin_lati);
        n_->getParam("origin_longitude", origin_longi);
        gps_sub_ = n_->subscribe<dji_sdk::GlobalPosition>("/dji_sdk/global_position", 1 ,&Multidrone::GpsCallback,this);


    }

    void GpsCallback(dji_sdk::GlobalPosition gps_data)
    {
        double lati = gps_data.latitude * PI/180;
        double longi = gps_data.longitude * PI/180;

        /* From GPS to Ground */
        double dlati = lati-origin_lati;
        double dlongti= longi-origin_longi;
        double x = dlati * C_EARTH;
        double y = dlongti * C_EARTH * cos(lati / 2.0 + origin_lati / 2.0);
        std::cout << x << "," << y << std::endl;

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
