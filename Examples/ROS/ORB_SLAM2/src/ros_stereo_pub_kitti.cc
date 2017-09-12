/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/TransformStamped.h"
#include<tf/tf.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/PoseStamped.h>

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    // ros::Publisher* ppath_pub;
    // nav_msgs::Path* ppath;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 settings_stereo do_rectify" << endl;
        ros::shutdown();
        return 1;
    }  
        //const string &strSettingPath = "Setting_stereo.yaml";
    const string &strSettingPath = string(argv[1]);
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    if(!fSettings.isOpened())
    {
        cerr << "Failed to open setting file at: " << strSettingPath << endl;
        exit(-1);
    }
    const string strORBvoc = fSettings["Orb_Vocabulary"];
    const string strCamSet = fSettings["Cam_Setting"];
    int ReuseMap = fSettings["is_ReuseMap"];
    int image_number=fSettings["image_number"];
    int start_number=fSettings["start"];
    int save_map=fSettings["save_map"];
    const string strMapPath = fSettings["ReuseMap"];
    const string strSquence = fSettings["Sequence"];
    //cout<<"can_load"<<endl;
    bool save_map_bool=false;
    if (1==save_map)
        save_map_bool=true;

    bool bReuseMap = false;
    if (1 == ReuseMap)
        bReuseMap = true;  

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(strORBvoc,strCamSet,ORB_SLAM2::System::STEREO,true,bReuseMap,strMapPath);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[2]);
	ss >> boolalpha >> igb.do_rectify;
//don't need do_rectift for kitti dataset
    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cout<<"_________________need do rectify"<<endl;
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;
    //change using path
    // ros::Publisher path_pub=nh.advertise<nav_msgs::Path>("trajectory",1,true);
    // igb.ppath_pub=&(path_pub);

    // nav_msgs::Path path;
    // igb.ppath=&(path);
    // ros::Time current_time;
    // current_time=ros::Time::now();
    // igb.path.header.stamp=current_time;
    // igb.path.header.frame_id="world";

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
     if(save_map_bool)
        SLAM.SaveMapRequest();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();
    if(ReuseMap)
         SLAM.Savenavigation("navigation.txt");

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat pose;

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        pose=mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        pose=mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

    // {
    //     /* global left handed coordinate system */
    // static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    // static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    // // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    // static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
    //                                                            -1, 1,-1, 1,
    //                                                            -1,-1, 1, 1,
    //                                                             1, 1, 1, 1);

    // //prev_pose * T = pose
    // cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
    // world_lh = world_lh * translation;
    // pose_prev = pose.clone();


    // /* transform into global right handed coordinate system, publish in ROS*/
    // tf::Matrix3x3 cameraRotation_rh(  - world_lh.at<float>(0,0),   world_lh.at<float>(0,1),   world_lh.at<float>(0,2),
    //                               - world_lh.at<float>(1,0),   world_lh.at<float>(1,1),   world_lh.at<float>(1,2),
    //                                 world_lh.at<float>(2,0), - world_lh.at<float>(2,1), - world_lh.at<float>(2,2));

    // tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

    // //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    // const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
    //                                         0, 0, 1,
    //                                         1, 0, 0);

    // static tf::TransformBroadcaster br;

    // tf::Matrix3x3 globalRotation_rh = cameraRotation_rh * rotation270degXZ;
    // tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;
    // tf::Transform transform = tf::Transform(globalRotation_rh, globalTranslation_rh);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "camera_pose"));
    // }
         /* global left handed coordinate system */
    static cv::Mat pose_prev = cv::Mat::eye(4,4, CV_32F);
    static cv::Mat world_lh = cv::Mat::eye(4,4, CV_32F);
    // matrix to flip signs of sinus in rotation matrix, not sure why we need to do that
    static const cv::Mat flipSign = (cv::Mat_<float>(4,4) <<   1,-1,-1, 1,
            -1, 1,-1, 1,
            -1,-1, 1, 1,
            1, 1, 1, 1);

    //prev_pose * T = pose
    cv::Mat translation =  (pose * pose_prev.inv()).mul(flipSign);
    world_lh = world_lh * translation;
    pose_prev = pose.clone();

    tf::Matrix3x3 tf3d;
    tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
            pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
            pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));

    tf::Vector3 cameraTranslation_rh( world_lh.at<float>(0,3),world_lh.at<float>(1,3), - world_lh.at<float>(2,3) );

    //rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
    const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
            0, 0, 1,
            1, 0, 0);

    static tf::TransformBroadcaster br;

    tf::Matrix3x3 globalRotation_rh = tf3d;
    tf::Vector3 globalTranslation_rh = cameraTranslation_rh * rotation270degXZ;

    tf::Quaternion tfqt;
    globalRotation_rh.getRotation(tfqt);

    double aux = tfqt[0];
    tfqt[0]=-tfqt[2];
    tfqt[2]=tfqt[1];
    tfqt[1]=aux;

    tf::Transform transform;
    transform.setOrigin(globalTranslation_rh);
    transform.setRotation(tfqt);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "camera_pose"));
    //change publish path
    //nav_msgs::Path path;
    // ros::Time current_time;
    // current_time=ros::Time::now();
    // path->header.stamp=current_time;
    // path->header.frame_id="world";

    // geometry_msgs::PoseStamped pose_stamped;
    // pose_stamped.pose.position.x=transform.getOrigin().x();
    // pose_stamped.pose.position.y=transform.getOrigin().y();
    // pose_stamped.pose.position.z=transform.getOrigin().z();
    // pose_stamped.pose.orientation.x=transform.getRotation().x();
    // pose_stamped.pose.orientation.y=transform.getRotation().y();
    // pose_stamped.pose.orientation.z=transform.getRotation().z();
    // pose_stamped.pose.orientation.w=transform.getRotation().w();
    // pose_stamped.header.stamp=current_time;
    // pose_stamped.header.frame_id="world";
    // ppath->poses.push_back(pose_stamped);
    // ppath_pub->publish(ppath);

}


