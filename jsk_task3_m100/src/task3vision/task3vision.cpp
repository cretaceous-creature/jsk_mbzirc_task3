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
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/core/core.hpp>

//msg headers.
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_task3_m100/ProjectionMatrix.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
//for test
#include <tf/transform_broadcaster.h>
#include <ctime>




class task3_vision
{
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry> MySyncPolicy;

private:

    message_filters::Subscriber<sensor_msgs::Image> *img_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> *camera_info_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *uav_odom_sub_;
    ros::NodeHandle nh_;
    ros::NodeHandle *priv_nh; //private
    message_filters::Synchronizer<MySyncPolicy> *sync;
    tf::Transform BaseToCamera;
    geometry_msgs::PoseArray object_clusteres;
    geometry_msgs::PoseArray object_clusteres_local;
    ros::Publisher param_matrix_pub_;
    ros::Publisher obj_clus_pub_;
    ros::Publisher obj_clus_pub_local_;
    //task3
    ros::Subscriber guidance_sub_;
    ros::Subscriber lidar_sub_;
    sensor_msgs::LaserScan us_data;
    float lidar_data;
    ros::Publisher filtered_img_pub;
    ros::Publisher label_img_pub;
    //dynamic reconfigure
    dynamic_reconfigure::Server<jsk_mbzirc_tasks::FilterparamConfig> server;
    dynamic_reconfigure::Server<jsk_mbzirc_tasks::FilterparamConfig>::CallbackType f;
    //dynamic reconfigure parameters
    int s_max,v_max,s_min,v_min;
    int r_off,g_off,b_off,ye_off,or_off;
    bool debug_show;
    int cluster_thre_dist;
    int downsample_cluster_size, min_cluster_size;
    double fake_dist;
    //param
    std::vector<std::vector<cv::Point> > clusters;
#define HSVRED -5
#define HSVGREEN 50 //because our color is more like blue....lol
#define HSVBLUE 120
#define HSVYELLOW 30
#define HSVORANGE 20
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
#define Ground_Z 0.2
    //test
    tf::TransformBroadcaster br;
public:
    void Init()
    {
        priv_nh = new ros::NodeHandle("~");
        std::string image_topic, cam_info_topic;
        priv_nh->param("image_topic", image_topic, std::string("/image_zenmus"));
        priv_nh->param("cam_info_topic", cam_info_topic, std::string("/dji_sdk/camera_info"));

        img_sub_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_,image_topic,1);
        camera_info_sub_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,cam_info_topic, 1);
        uav_odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,"/dji_sdk/odometry",1);
        lidar_sub_ = nh_.subscribe<std_msgs::Float32>("/lidar_laser",1,&task3_vision::LidarCallback,this);
        guidance_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/guidance/ultrasonic",1,&task3_vision::GuidanceCallback,this);
        sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *img_sub_, *camera_info_sub_, *uav_odom_sub_);
        sync->registerCallback(boost::bind(&task3_vision::ImageCallback,this,_1,_2,_3));
        std::string topic2 = nh_.resolveName("projection_matrix");
        param_matrix_pub_ = nh_.advertise<jsk_task3_m100::ProjectionMatrix>(topic2,1);

        obj_clus_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/obj_cluster/centroid_pose",1);
        obj_clus_pub_local_ = nh_.advertise<geometry_msgs::PoseArray>("/obj_cluster/centroid_pose_local",1);
        filtered_img_pub = nh_.advertise<sensor_msgs::Image>("/filtered_all",1);
        label_img_pub = nh_.advertise<sensor_msgs::Image>("/label_img",1);

        /*** dynamic reconfigure ***/
        f = boost::bind(&task3_vision::DynamicReconfigureCallback, this, _1,_2);
        server.setCallback(f);
        cluster_thre_dist = 30;
        downsample_cluster_size =1000;
        min_cluster_size = 10;
        fake_dist = 0;

        //initialize base_link to camera optical link
        BaseToCamera.setOrigin(tf::Vector3(0,0,0));
        BaseToCamera.setRotation(tf::Quaternion(0.707, -0.707, 0.000, -0.000)); //facing down..
        std::cout<<"initialization finished"<<std::endl;

    }
    void LidarCallback(const std_msgs::Float32 data)
    {
        lidar_data = (float)data.data;
        lidar_data /= 100; //from cm to meter
    }

    void GuidanceCallback(const sensor_msgs::LaserScan data)
    {
        us_data = data;
    }

    void ImageCallback(const sensor_msgs::ImageConstPtr& img,
                       const sensor_msgs::CameraInfoConstPtr& cam_info,
                       const nav_msgs::OdometryConstPtr& odom);
    void DynamicReconfigureCallback(jsk_mbzirc_tasks::FilterparamConfig &config, uint32_t level)
    {
        s_max = config.hsv_s_max; v_max = config.hsv_v_max;
        s_min = config.hsv_s_min; v_min = config.hsv_v_min;
        r_off = config.r_offset;g_off = config.g_offset;b_off = config.b_offset;
        ye_off = config.ye_offset;or_off = config.or_offset;
        debug_show = config.debug_show;
        fake_dist = config.fake_height;

        cluster_thre_dist = config.cluster_threhold_dist;
        downsample_cluster_size = config.downsample_cluster_size;
        min_cluster_size = config.min_cluster_size;
    }

    //clustering of one filtered image
    std::vector<std::vector<cv::Point> > EuclideanCluster(
            cv::Mat *image_input, cv::Mat3b *lbl, int n_factor, char c);
    //projection matrix
    cv::Mat Projection_matrix(const sensor_msgs::CameraInfoConstPtr& cam_info,
                              const nav_msgs::OdometryConstPtr& odom, double uav_h);

    // class's family...
     //task3_vision(const ros::NodeHandle &nh = ros::NodeHandle(), const ros::NodeHandle &priv_nh = ros::NodeHandle("~"))
    ~task3_vision()
    {    }
};

//calculate projection matrix
cv::Mat task3_vision::Projection_matrix(const sensor_msgs::CameraInfoConstPtr& cam_info,
                                        const nav_msgs::OdometryConstPtr& odom,
                                        double uav_h)
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
    {  //skip local position, directly use uav center coordinate
        //    tf::poseMsgToTF(odom->pose.pose,tfpose);
        //    tf::Matrix3x3 rot = tfpose.getBasis();
        //    tfScalar yaw, pitch, roll;
        //    rot.getEulerYPR(yaw, pitch, roll);
        //    rot.setEulerYPR(yaw,0,0);
        //    tfpose.setBasis(rot);
    }
    BaseToCamera.setOrigin(tf::Vector3(0,0.05,uav_h));
    extrisic = BaseToCamera;//*tfpose.inverse();
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

    return P_Mat_G;

}


//find zeros function
inline std::vector<cv::Point> DownSamplePoints(cv::Mat *img_input, int n)
{
    std::vector<cv::Point> pts_raw;
    std::vector<cv::Point> pts;
    if(cv::countNonZero(*img_input))
    {
        findNonZero(*img_input, pts_raw);
        //clustered n_labels objects, however the problem is when getting near,
        //too many points to be calculated, need to downsample...
        //downsample the points if there are too much
        pts.resize(pts_raw.size()/n);
        for(long i = 0; i < pts.size(); i+=n)
        {
            pts[i] = pts_raw[i*n];
        }
    }
    return pts;
}

//Euclideanclustering function
std::vector<std::vector<cv::Point> > task3_vision::EuclideanCluster(
        cv::Mat *image_input, cv::Mat3b *lbl, int n_factor, char c)
{
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Point> pts = DownSamplePoints(image_input, n_factor);

    if(pts.size())
    {
        std::vector<int> labels;
        int n_labels = cv::partition(pts, labels, EuclideanDistanceFunctor(cluster_thre_dist));
        contours.resize(n_labels);  //contours are the clusteres with points.
        for (int i = 0; i < pts.size(); ++i)
        {
            if(pts[i].x!=0 && pts[i].y!=0)
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
                    break;
            }
            else
            {
                //get the centroid of each contours
                geometry_msgs::Pose tmp;
                for(int i = 0; i<itr->size(); i++)
                {
                    tmp.position.x += itr->at(i).x;
                    tmp.position.y += itr->at(i).y;
                }
                tmp.position.x /= itr->size();
                tmp.position.y /= itr->size();
                // pose orientation: x: points size, y: color, z orientation
                tmp.orientation.x = itr->size();
                tmp.orientation.y = c;
                object_clusteres.poses.push_back(tmp);
            }
        }
        int label_size = contours.size();
        if(debug_show)
            std::cout<<"child points size is:" <<pts.size()<<std::endl<< "clusters are::" << label_size <<std::endl;
        for (int i = 0; i < label_size; i++)
        {
            cv::Scalar color;
            switch(c)
            {
            case 'a':
                color = cv::Scalar(rand() & 255, rand() & 255, rand() & 255);
                break;
            case 'r':
                color = cv::Scalar(0, 0, 255);
                break;
            case 'g':
                color = cv::Scalar(0, 255, 0);
                break;
            case 'b':
                color = cv::Scalar(255, 0, 0);
                break;
            case 'y':
                color = cv::Scalar(0, 255, 255);
                break;
            case 'o':
                color = cv::Scalar(0, 130, 255);
                break;
            default:
                break;
            }
            for(int j = 0; j< contours[i].size(); j++)
            {
                cv::circle(*lbl, contours[i].at(j), 1, color, -1, 8, 0 );
            }
        }
    }
    return contours;
}



/****
 * Image call back function
****/
void task3_vision::ImageCallback(const sensor_msgs::ImageConstPtr& img,
                                 const sensor_msgs::CameraInfoConstPtr& cam_info,
                                 const nav_msgs::OdometryConstPtr& odom)
{
    double uav_h;
    if(odom->pose.pose.position.z<1&&us_data.ranges.size()&&us_data.ranges.at(0)>0.05) //less than 1 meter
        uav_h = us_data.ranges.at(0) - 0.05;
    else if(odom->pose.pose.position.z<1&&lidar_data>0&&lidar_data<2) //less than 1 meter
        uav_h = lidar_data;
    else
        uav_h = odom->pose.pose.position.z;

    try
    {
        cv::Mat raw_image = cv_bridge::toCvCopy(img,"bgr8")->image;
        cv::Mat hsv_image;
        cv::Mat hsv_filtered_r, hsv_filtered_g, hsv_filtered_b, hsv_filtered_ye, hsv_filtered_or, hsv_filtered_all;
        //assign the dist threshold by considering the height...
        cluster_thre_dist = 50 / (fake_dist+0.1);
        cluster_thre_dist = 50 / (uav_h + 0.1);
        cluster_thre_dist = cluster_thre_dist<10?10:cluster_thre_dist;
        /***********counting the timer********/
        std::clock_t start;
        double duration;
        start = std::clock();

        /**********first HSI Filter**********/
        {
            cv::cvtColor(raw_image,hsv_image,cv::COLOR_BGR2HSV);
            cv::Mat hsv_r_down;
            //r
            cv::inRange(hsv_image, cv::Scalar(180+HSVRED-r_off, s_min, v_min, 0), cv::Scalar(180, s_max, v_max, 0), hsv_filtered_r);
            cv::inRange(hsv_image, cv::Scalar(0, s_min, v_min, 0), cv::Scalar(r_off+HSVRED, s_max, v_max, 0), hsv_r_down);
            hsv_filtered_r |=hsv_r_down;
            //g
            cv::inRange(hsv_image, cv::Scalar(HSVGREEN-g_off, s_min, v_min, 0), cv::Scalar(HSVGREEN+g_off, s_max, v_max, 0), hsv_filtered_g);
            //b
            cv::inRange(hsv_image, cv::Scalar(HSVBLUE-b_off, s_min, v_min, 0), cv::Scalar(HSVBLUE+b_off, s_max, v_max, 0), hsv_filtered_b);
            //ye
            cv::inRange(hsv_image, cv::Scalar(HSVYELLOW-ye_off, s_min, v_min, 0), cv::Scalar(HSVYELLOW+ye_off, s_max, v_max, 0), hsv_filtered_ye);
            //or
            cv::inRange(hsv_image, cv::Scalar(HSVORANGE-or_off, s_min, v_min, 0), cv::Scalar(HSVORANGE+or_off, s_max, v_max, 0), hsv_filtered_or);
        }
        //show image of r g b ye or
        hsv_filtered_all = hsv_filtered_r | hsv_filtered_g | hsv_filtered_b |  hsv_filtered_ye |  hsv_filtered_or;

        if(debug_show)
        {
            cv::imshow("all space",hsv_filtered_all);
            cv::imshow("red space",hsv_filtered_r);
            cv::imshow("green space",hsv_filtered_g);
            cv::imshow("blue space",hsv_filtered_b);
            cv::imshow("yellow space",hsv_filtered_ye);
            cv::imshow("orange space",hsv_filtered_or);
        }
        else
        {
            cv::destroyWindow("red space");cv::destroyWindow("green space");cv::destroyWindow("blue space");
            cv::destroyWindow("yellow space");cv::destroyWindow("orange space");cv::destroyWindow("all space");
        }

        /**********Then Euclidean Clustering**********/
        //clustering..
        /***
         * Two methods:
         * 1: combine all and do clustering
         * 2: Every filter do clustering
         * 3: Findcontours
         * 4: Hough Circle Transform
         ***/
        int ptr_all = cv::countNonZero(hsv_filtered_r) + cv::countNonZero(hsv_filtered_g)
                + cv::countNonZero(hsv_filtered_b) + cv::countNonZero(hsv_filtered_ye)
                + cv::countNonZero(hsv_filtered_or);
        //n_ds is the downsample factor
        int n_ds = ptr_all/downsample_cluster_size + 1; //begin from  1
        if(debug_show)
            std::cout<<"All points num are: "<<ptr_all<<std::endl;

        //clear the previous data
        this->clusters.clear();
        this->object_clusteres.poses.clear();
        this->object_clusteres.header = odom->header;
        // Build a vector of random color, one for each class (label) and draw the labels
        cv::Mat3b lbl(hsv_filtered_all.rows, hsv_filtered_all.cols, cv::Vec3b(0, 0, 0));
        //method 1:
        //         this->clusters = this->EuclideanCluster(&hsv_filtered_all, &lbl, n_ds, 'a');
        //method 2:
        {
            this->clusters = this->EuclideanCluster(&hsv_filtered_r, &lbl, n_ds, 'r');
            this->EuclideanCluster(&hsv_filtered_g, &lbl, n_ds, 'g');
            this->EuclideanCluster(&hsv_filtered_b, &lbl, n_ds, 'b');
            //because of the color of background, no orange..

            this->EuclideanCluster(&hsv_filtered_ye, &lbl, n_ds, 'y');
            //this->EuclideanCluster(&hsv_filtered_or, &lbl, n_ds, 'o');
        }
        //method 3:
        {
            //            std::vector<cv::Vec4i> hierarchy;
            //            std::vector<std::vector<cv::Point> > contours;
            //            cv::Mat canny_output;
            //            /// Detect edges using canny
            //            Canny( hsv_filtered_all, canny_output, 100, 100, 3 );
            //            cv::findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
            //            for( int i = 0; i< contours.size(); i++ )
            //            {
            //                cv::Scalar color;
            //                color = cv::Scalar(rand() & 255, rand() & 255, rand() & 255);
            //                drawContours(lbl, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
            //            }
        }
        //method 4:
        {
            //            std::vector<cv::Vec3f> circles;
            //            /// Apply the Hough Transform to find the circles
            //            cv::HoughCircles( hsv_filtered_all, circles, CV_HOUGH_GRADIENT,
            //                              1, 10, 100, 30, 1, 200);
            //            /// Draw the circles detected
            //            for( size_t i = 0; i < circles.size(); i++ )
            //            {
            //                cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            //                int radius = cvRound(circles[i][2]);
            //                cv::Scalar color;
            //                color = cv::Scalar(rand() & 255, rand() & 255, rand() & 255);
            //                // circle center
            //                cv::circle(lbl, center, 3, color, -1, 8, 0 );
            //                // circle outline
            //                cv::circle(lbl, center, radius, color, 3, 8, 0 );
            //             }
        }

        for (int i = 0; i < object_clusteres.poses.size(); i++)
        {
            cv::Scalar color;
            char c = (char)object_clusteres.poses.at(i).orientation.y;
            switch(c)
            {
            case 'a':
                color = cv::Scalar(rand() & 255, rand() & 255, rand() & 255);
                break;
            case 'r':
                color = cv::Scalar(0, 0, 255);
                break;
            case 'g':
                color = cv::Scalar(0, 255, 0);
                break;
            case 'b':
                color = cv::Scalar(255, 0, 0);
                break;
            case 'y':
                color = cv::Scalar(0, 255, 255);
                break;
            case 'o':
                color = cv::Scalar(0, 130, 255);
                break;
            default:
                break;
            }
            cv::circle(lbl,
                       cv::Point((int)object_clusteres.poses.at(i).position.x,
                                 (int)object_clusteres.poses.at(i).position.y),
                       (int)object_clusteres.poses.at(i).orientation.x/2, color, 10, 8, 0 );
        }
        cv::Mat test;
        cv::cvtColor(lbl,test,cv::COLOR_BGR2GRAY);
        if(debug_show)
        {
            std::cout<<"label points are: "<< cv::countNonZero(test)<<std::endl;
            imshow("Labels", lbl);
        }

        cv::Mat P = Projection_matrix(cam_info,odom,uav_h);
        double a[4],b[4],c[4];
        {
            a[0] = P.at<double>(0, 0);
            a[1] = P.at<double>(0, 1);
            a[2] = P.at<double>(0, 2);
            a[3] = P.at<double>(0, 3);
            b[0] = P.at<double>(1, 0);
            b[1] = P.at<double>(1, 1);
            b[2] = P.at<double>(1, 2);
            b[3] = P.at<double>(1, 3);
            c[0] = P.at<double>(2, 0);
            c[1] = P.at<double>(2, 1);
            c[2] = P.at<double>(2, 2);
            c[3] = P.at<double>(2, 3);
        }
        //transfrom to world coordinate, just plus the drone coordinate
        //since we ignore the pitch and roll, just use the yaw
        //double yaw = odom->pose.pose.orientation
        tf::Quaternion q(odom->pose.pose.orientation.x,
                         odom->pose.pose.orientation.y,
                         odom->pose.pose.orientation.z,
                         odom->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double r,p,y;
        m.getEulerYPR(y,p,r);
        object_clusteres_local = object_clusteres;
        for (int i = 0; i < object_clusteres.poses.size(); i++)
        {
            float A[2][2],bv[2];
            float j = object_clusteres.poses[i].position.x;
            float k = object_clusteres.poses[i].position.y;
            //j is the height and k is the width
            A[0][0] = j*c[0] - a[0]; A[0][1] = j*c[1] - a[1];
            A[1][0] = k*c[0] - b[0]; A[1][1] = j*c[1] - b[1];
            bv[0]= a[2]*Ground_Z + a[3] - j*c[2]*Ground_Z - j*c[3];
            bv[1] = b[2]*Ground_Z + b[3] - k*c[2]*Ground_Z - k*c[3];
            float DomA = A[1][1]*A[0][0]-A[0][1]*A[1][0];
            double local_x = (A[1][1]*bv[0]-A[0][1]*bv[1])/DomA;
            double local_y = (A[0][0]*bv[0]-A[1][0]*bv[0])/DomA;
            //local position
            object_clusteres_local.poses[i].position.x = local_x;
            object_clusteres_local.poses[i].position.y = local_y;
            //global position
            object_clusteres.poses[i].position.x = odom->pose.pose.position.x +
                    local_x * std::cos(y) + local_y * std::sin(y);
            object_clusteres.poses[i].position.y = odom->pose.pose.position.y -
                    local_y * std::cos(y) + local_x * std::sin(y);
            object_clusteres.poses[i].position.z = Ground_Z;


        }

        //publish the cluster position
        obj_clus_pub_.publish(object_clusteres);
        obj_clus_pub_local_.publish(object_clusteres_local);

        cv_bridge::CvImage filtered_ros_img;
        filtered_ros_img.header = img->header;
        filtered_ros_img.encoding = "mono8";
        filtered_ros_img.image = hsv_filtered_all;
        filtered_img_pub.publish(filtered_ros_img.toImageMsg());
        filtered_ros_img.encoding = "bgr8";
        filtered_ros_img.image = lbl;
        label_img_pub.publish(filtered_ros_img.toImageMsg());

        //timer end
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        //std::cout<<"process_time is "<< duration << " second" <<'\n';

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
    u_i2p.Init();

    ros::spin();

}
