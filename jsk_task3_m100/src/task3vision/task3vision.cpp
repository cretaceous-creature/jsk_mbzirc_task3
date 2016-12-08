/*
 jsk_mbzirc_task
 */

// Author: Chen

//ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_task3_m100/FilterparamConfig.h>

#include <string.h>
#include <vector>
//OPENCV
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/cudafilters.hpp>
//#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core/core.hpp>

//msg headers.
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
//#include <jsk_mbzirc_msgs/ProjectionMatrix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
//for test
#include <tf/transform_broadcaster.h>
#include <ctime>




class task3_vision
{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,nav_msgs::Odometry> MySyncPolicy;

private:

    message_filters::Subscriber<sensor_msgs::Image> *img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *uav_odom_sub_;
    ros::NodeHandle nh_;
    message_filters::Synchronizer<MySyncPolicy> *sync;
    tf::Transform BaseToCamera;
    geometry_msgs::PoseArray object_clusteres;

    ros::Publisher obj_clus_pub_;
    //task3
    ros::Subscriber pick_state_sub_;
    std_msgs::Bool pick_state;
    //dynamic reconfigure
    dynamic_reconfigure::Server<jsk_mbzirc_tasks::FilterparamConfig> server;
    dynamic_reconfigure::Server<jsk_mbzirc_tasks::FilterparamConfig>::CallbackType f;
    int h_max,s_max,v_max,h_min,s_min,v_min;
    int cluster_thre_dist;
    int downsample_cluster_size, min_cluster_size;
    //functor
    struct EuclideanDistanceFunctor
    {
        int _dist2;
        EuclideanDistanceFunctor(int dist) : _dist2(dist*dist) {}

        bool operator()(const cv::Point& lhs, const cv::Point& rhs) const
        {
            return ((lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y)) < _dist2;
        }
    };
#define Ground_Z 0.0
   //test
    tf::TransformBroadcaster br;
public:
    void init()
    {
        img_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_,"/downward_cam/camera/image",2);
        camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,"/downward_cam/camera/camera_info", 2);
        uav_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,"/ground_truth/state",2);
        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(30), *img_sub_, *camera_info_sub_, *uav_odom_sub_);
        sync->registerCallback(boost::bind(&task3_vision::imageCallback,this,_1,_2,_3));

        obj_clus_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/obj_cluster/centroid_pose",1);
        /*
        if(!nh_.getParam("enableGPU",GPUFLAG))
            std::cout<<"fail to load the param enableGPU, Using CPU instead"<<std::endl;
        else
            std::cout<<"With GPU support flag = " << GPUFLAG<<std::endl;
            */

        /*** dynamic reconfigure ***/
        f = boost::bind(&task3_vision::dynamic_reconfigure_callback, this, _1,_2);
        server.setCallback(f);
        h_max=120; s_max=200; v_max=255;
        h_min=0;   s_min=101; v_min=80;
        cluster_thre_dist = 18;
        downsample_cluster_size = 1000;
        min_cluster_size = 10;

        //initialize base_link to camera optical link
        BaseToCamera.setOrigin(tf::Vector3(0,0,-0.05));
        BaseToCamera.setRotation(tf::Quaternion(0.707, -0.707, 0.000, -0.000));


    }

    void imageCallback(const sensor_msgs::ImageConstPtr& img,
                       const sensor_msgs::CameraInfoConstPtr& cam_info,
                       const nav_msgs::OdometryConstPtr& odom);
    void dynamic_reconfigure_callback(jsk_mbzirc_tasks::FilterparamConfig &config, uint32_t level)
    {
      h_max = config.hsv_h_max; s_max = config.hsv_s_max; v_max = config.hsv_v_max;
      h_min = config.hsv_h_min; s_min = config.hsv_s_min; v_min = config.hsv_v_min;
      cluster_thre_dist = config.cluster_threhold_dist;
      downsample_cluster_size = config.downsample_cluster_size;
      min_cluster_size = config.min_cluster_size;
    }

    // class's family...
    ~task3_vision()
    {    }
};




void task3_vision::imageCallback(const sensor_msgs::ImageConstPtr& img,
                                       const sensor_msgs::CameraInfoConstPtr& cam_info,
                                       const nav_msgs::OdometryConstPtr& odom)
{
    try
    {
        cv::Mat raw_image = cv_bridge::toCvCopy(img,"bgr8")->image;
        cv::Mat hsv_image;
        cv::Mat hsv_filtered;
        /***********counting the timer********/
        std::clock_t start;
        double duration;
        start = std::clock();
        /**********first HSI Filter**********/
#ifdef GPU_EN
        cv::cuda::cvtColor(raw_image,hsv_image,cv::COLOR_BGR2HSV);

#else
        cv::cvtColor(raw_image,hsv_image,cv::COLOR_BGR2HSV);
        cv::inRange(hsv_image, cv::Scalar(h_min, s_min, v_min, 0), cv::Scalar(h_max, s_max, v_max, 0), hsv_filtered);
        cv::imshow("hsv_filtered",hsv_filtered);
#endif
        /**********Then Euclidean Clustering**********/
#ifdef GPU_EN
        cv::cuda::cvtColor(raw_image,hsv_image,cv::COLOR_BGR2HSV);

#else
        //clustering..
        std::vector<cv::Point> pts_raw;
        findNonZero(hsv_filtered, pts_raw);
        //clustered n_labels objects, however the problem is when getting near,
        //too many points to be calculated, need to downsample...
        //downsample the points if there are too much
        int n = pts_raw.size()/downsample_cluster_size + 1; //begin from  1
        std::vector<cv::Point> pts(pts_raw.size()/n);
        for(long i = 0; i < pts.size(); i+=n)
          {
            pts[i] = pts_raw[i*n];
          }

        std::vector<int> labels;

        int n_labels = cv::partition(pts, labels, EuclideanDistanceFunctor(cluster_thre_dist));
        std::vector<std::vector<cv::Point> > contours(n_labels);  //contours are the clusteres with points.
        for (int i = 0; i < pts.size(); ++i)
          {
            contours[labels[i]].push_back(pts[i]);
          }
        /****
         * labels vector gives every point a label by labels[pts[n]], we need a parameter to remove
         * the labels that only contains small amount of points which could be considered as noise.
        ****/
        //check contours size and remove the clusteres with less points... also get the center...
        for (std::vector<std::vector<cv::Point> >::iterator itr = contours.begin(); itr != contours.end(); ++itr)
          {
            if(itr->size() < min_cluster_size)
              {
                contours.erase(itr);
                if(itr == contours.end())
                  {
                    break;
                  }
              }
            else
              {
                //get the centroid of each contours

              }
          }

        // Build a vector of random color, one for each class (label)
        std::vector<cv::Vec3b> colors;
        for (int i = 0; i < n_labels; ++i)
          {
            colors.push_back(cv::Vec3b(rand() & 255, rand() & 255, rand() & 255));
          }

        // Draw the labels
        cv::Mat3b lbl(hsv_filtered.rows, hsv_filtered.cols, cv::Vec3b(0, 0, 0));
        for (int i = 0; i < pts.size(); ++i)
          {
            lbl(pts[i]) = colors[labels[i]];
          }

        imshow("Labels", lbl);
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        std::cout<<"process_time is "<< duration << " second" <<'\n';
        // cv::findContours();
        // cv::partition();
#endif

        cv::waitKey(10);
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task3_vision");
    task3_vision u_i2p;
    u_i2p.init();

    ros::spin();

}
