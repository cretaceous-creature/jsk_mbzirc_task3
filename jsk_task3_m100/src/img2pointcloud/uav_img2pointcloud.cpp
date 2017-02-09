/*
 jsk_mbzirc_task
 */

// Author: Chen

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
//msg headers.
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_task3_m100/ProjectionMatrix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <iostream>

//for test
#include <tf/transform_broadcaster.h>


class uav_img2pointcloud
{
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo,nav_msgs::Odometry> MySyncPolicy;

private:
    ros::Publisher param_matrix_pub_;
    //sensor_msgs::PointCloud2 cloud_msg;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *uav_odom_sub_;
    ros::NodeHandle nh_;
    message_filters::Synchronizer<MySyncPolicy> *sync;
    tf::Transform BaseToCamera;

   //test
    tf::TransformBroadcaster br;
public:
    void init()
    {
        //publish projection matrix
        std::string topic2 = nh_.resolveName("projection_matrix");
        param_matrix_pub_ = nh_.advertise<jsk_task3_m100::ProjectionMatrix>(topic2,1);

        camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,"/dji_sdk/camera_info", 1);
        uav_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,"/dji_sdk/odometry",1);
        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(30), *camera_info_sub_, *uav_odom_sub_);
        sync->registerCallback(boost::bind(&uav_img2pointcloud::ProjectionCallback,this,_1,_2));

        //initialize base_link to camera optical link
        BaseToCamera.setOrigin(tf::Vector3(0,0,0));
        BaseToCamera.setRotation(tf::Quaternion(0.707, -0.707, 0.000, -0.000)); //facing down..
    }
    //call back, for processing
    void ProjectionCallback(const sensor_msgs::CameraInfoConstPtr& cam_info,
                       const nav_msgs::OdometryConstPtr& odom);
    // class's family...
    ~uav_img2pointcloud()
    {    }
};






void uav_img2pointcloud::ProjectionCallback(const sensor_msgs::CameraInfoConstPtr& cam_info,
                                       const nav_msgs::OdometryConstPtr& odom)
{
    tf::Transform extrisic;
    cv::Mat P(3,4,CV_64FC1);
    cv::Mat P_Mat_G(3,4,CV_64FC1);
    tf::Pose tfpose;
    tfScalar extrisic_data[4*4];
    jsk_task3_m100::ProjectionMatrix param_vector;
    std_msgs::MultiArrayDimension dim;
    dim.size = 3;dim.label = "height";
    param_vector.layout.dim.push_back(dim);
    dim.size = 4;dim.label = "width";
    param_vector.layout.dim.push_back(dim);

    //get extrisic, to the world coordinate
    //since we use gimbal, set the pose orientation
    //always face down
    tf::poseMsgToTF(odom->pose.pose,tfpose);
    tf::Matrix3x3 rot = tfpose.getBasis();
    tfScalar yaw, pitch, roll;
    rot.getEulerYPR(yaw, pitch, roll);
    rot.setEulerYPR(yaw,0,0);
    tfpose.setBasis(rot);

    extrisic = BaseToCamera*tfpose.inverse();
    //pinv of projection matrix...
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 4; j++)
            P.at<double>(i,j) = cam_info->P.at(i*4+j);
    // however, this P is in camera coordinate..
    extrisic.getOpenGLMatrix(extrisic_data);
    cv::Mat E_MAT(4, 4, CV_64FC1, extrisic_data);
    P_Mat_G = P*(E_MAT.t());
    // now is the ground, namely, world coordinate
    double a[4], b[4], c[4];
    a[0] = P_Mat_G.at<double>(0, 0);
    a[1] = P_Mat_G.at<double>(0, 1);
    a[2] = P_Mat_G.at<double>(0, 2);
    a[3] = P_Mat_G.at<double>(0, 3);
    b[0] = P_Mat_G.at<double>(1, 0);
    b[1] = P_Mat_G.at<double>(1, 1);
    b[2] = P_Mat_G.at<double>(1, 2);
    b[3] = P_Mat_G.at<double>(1, 3);
    c[0] = P_Mat_G.at<double>(2, 0);
    c[1] = P_Mat_G.at<double>(2, 1);
    c[2] = P_Mat_G.at<double>(2, 2);
    c[3] = P_Mat_G.at<double>(2, 3);



            //directly calculation make it faster, next step is to parallize it
    /*
            float A[2][2],bv[2];
            A[0][0] = j*c[0] - a[0]; A[0][1] = j*c[1] - a[1];
            A[1][0] = i*c[0] - b[0]; A[1][1] = i*c[1] - b[1];
            bv[0]= a[2]*Ground_Z + a[3] - j*c[2]*Ground_Z - j*c[3];
            bv[1] = b[2]*Ground_Z + b[3] - i*c[2]*Ground_Z - i*c[3];
            float DomA = A[1][1]*A[0][0]-A[0][1]*A[1][0];
            Pointcloud.points[i*Pointcloud.width+j].x = (A[1][1]*bv[0]-A[0][1]*bv[1])/DomA;
            Pointcloud.points[i*Pointcloud.width+j].y = (A[0][0]*bv[1]-A[1][0]*bv[0])/DomA;
            Pointcloud.points[i*Pointcloud.width+j].z = (float)Ground_Z;
    */

    //publish param matrix 4*3 = 12
    param_vector.header = cam_info->header;
    for(int i = 0; i < 4; i++)
        param_vector.data.push_back(a[i]);
    for(int i = 0; i < 4; i++)
        param_vector.data.push_back(b[i]);
    for(int i = 0; i < 4; i++)
        param_vector.data.push_back(c[i]);

    param_matrix_pub_.publish(param_vector);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_to_pointcloud");
    uav_img2pointcloud u_i2p;
    u_i2p.init();
    ros::spin();
}
