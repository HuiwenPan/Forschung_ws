/**************************
ROS HEADER
**************************/
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PointStamped.h>

/**************************
STD HEADER
**************************/
#include <vector>
#include <stdio.h>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>

/**************************
tf HEADER
**************************/
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

/**************************
OpenCV HEADER
**************************/
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core/core.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

/**************************
PCL HEADER
**************************/
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

using namespace cv;
using namespace std;

class ArucoDetector{
    private:
        ros::NodeHandle nh_;
        Mat cameraMatrix_, distCoeffs_;
        tf::TransformBroadcaster Marker_br_;
        ros::Subscriber image_sub;
        ros::Subscriber pcl_sub;
        vector<geometry_msgs::PointStamped> center_;
        vector<geometry_msgs::PointStamped> center_3d;

    public:
        ArucoDetector(ros::NodeHandle n):nh_(n) {
            double fx,fy,cx,cy,k1,k2,k3,p1,p2;
            fx=540.462622;
            fy=540.448667;
            cx=321.209797;
            cy=248.175096;
            k1=0.041615;
            k2=-0.113986;
            k3=0;
            p1=0.006056;
            p2=0.005122;
            cameraMatrix_ = (cv::Mat_<float>(3, 3) <<    fx, 0.0, cx,
                                                0.0, fy, cy,
                                                0.0, 0.0, 1.0);
        
            distCoeffs_ = (cv::Mat_<float>(5, 1) << k1, k2, p1, p2, k3);

            image_sub = nh_.subscribe("/camera/rgb/image_rect_color", 1, &ArucoDetector::Image_process, this);
            pcl_sub = nh_.subscribe("/camera/depth_registered/points", 1, &ArucoDetector::PC_process, this);
        } //Constructor

        ~ArucoDetector(){}

        void PC_process(const sensor_msgs::PointCloud2ConstPtr & pc);

        void Image_process(const sensor_msgs::Image::ConstPtr& msg);

        void createArucoMarkers();


}; //ArucoDetector def

void ArucoDetector::Image_process(const sensor_msgs::Image::ConstPtr& msg){


    cv_bridge::CvImagePtr cv_ptr;


    tf::Transform transform;

    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
    }

    namedWindow("Aruco_detector", CV_WINDOW_AUTOSIZE);
    Mat image, image_copy;
    image = cv_ptr->image;
    image.copyTo(image_copy);
   //imshow("image",image);
   
   vector<int> markerIds;
   vector< vector< Point2f > > markerCorners, rejectedCandidates;
   Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
   Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
   aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
   if(markerIds.size() > 0){
       ROS_INFO("get markers");
       aruco::drawDetectedMarkers(image_copy, markerCorners, markerIds);
       std::vector<cv::Vec3d> rvecs, tvecs;
       aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix_, distCoeffs_, rvecs, tvecs);//求解旋转矩阵rvecs和平移矩阵tvecs

       geometry_msgs::PointStamped point_temp;
       center_.clear();
       for(int i=0; i<markerIds.size(); i++){
               aruco::drawAxis(image_copy, cameraMatrix_, distCoeffs_, rvecs[i], tvecs[i], 0.1);

               point_temp.header.stamp = ros::Time::now();
               point_temp.header.frame_id = "/Marker";
               point_temp.point.x = (markerCorners[i][0].x + markerCorners[i][1].x) /2;
               point_temp.point.y = (markerCorners[i][0].y + markerCorners[i][3].y) /2;
               center_.push_back(point_temp);

               ROS_INFO("R: %f, T: %f", rvecs[i][0], tvecs[i][0]);
               transform.setOrigin(tf::Vector3(tvecs[0][0],tvecs[0][1],tvecs[0][2]));
               tf::Quaternion q;
               q.setRPY(rvecs[0][0],rvecs[0][1],rvecs[0][2]);
               transform.setRotation(q);
               Marker_br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/Marker", "/camera_rgb_frame"));

       }
   }
   else{
       ROS_INFO("NO markers");

   }
   imshow("out", image_copy);
   waitKey(10);
    
}


void ArucoDetector::PC_process(const sensor_msgs::PointCloud2ConstPtr & pc_temp){

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromROSMsg(*pc_temp, pc);
    geometry_msgs::PointStamped point_temp;
    for (auto center: center_){
        point_temp.point.x = pc.at(center.point.x, center.point.y).x;
        point_temp.point.y = pc.at(center.point.x, center.point.y).y;
        point_temp.point.z = pc.at(center.point.x, center.point.y).z;
        point_temp.header.stamp = center.header.stamp;
        point_temp.header.frame_id = center.header.frame_id;
        center_3d.push_back(point_temp);
    }



}


void ArucoDetector::createArucoMarkers(){

    Mat outputMarker;

    Ptr<aruco::Dictionary> markerDicitonary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    for(int i=0; i<50; i++){
        aruco::drawMarker(markerDicitonary, i, 500, outputMarker, 1);
        ostringstream convert;
        string imageName = "4X4Marker_";
        convert << imageName << i << ".jpg";
        imwrite(convert.str(), outputMarker);
    }

}








//void createKnownBoardPosition()

int main(int argc, char** argv){
    
    ros::init(argc, argv, "aruco_detector");

    ros::MultiThreadedSpinner spinner(3);
    ros::NodeHandle n;  
    ArucoDetector aruco_detector(n);
    spinner.spin();
    //createArucoMarkers();

    return 0;    
    

    


}