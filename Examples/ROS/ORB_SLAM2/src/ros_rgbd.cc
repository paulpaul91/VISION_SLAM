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

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

#include<opencv2/core/core.hpp>
#include "Converter.h"

#include"../../../include/System.h"
#include "tf/transform_datatypes.h"
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2/LinearMath/Transform.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;



/* * \ Mainpage ORB-SLAM2 commented and modified, personal fork os1
 * \ Section intro_sec Introduction to ORB-SLAM2
 * ORB-SLAM2 is version 2 of ORB-SLAM, published the following year of the original by the same authors.
 * This is a proof of concept, an application that demonstrates the operation of ORB-SLAM.
 * The application has a minimal and optional user interface.
 * ORB-SLAM2 is intended to be used as part of a larger project, connected by ROS.
 *
 * \ Section conf Configuration for first use
 * In order to compile this code with Eclipse CDT, OpenCV 2.4.3 or higher is required, including 3.1,
* Pangolin, Blas, Lapack, Eigen3. Consult https://github.com/raulmur/ORB_SLAM2 2.Prerequisites.
 * In Eclipse these libraries must be loaded (Properties for os1, Settings, g ++ Linker Libraries):
 *
 * - opencv_core, used for Mat
 * - pangolin, for map display
 * - opencv_features2d, for FAST
 * - opencv_highgui, to display the camera frame
 * - pthread, to trigger threads
 * - opencv_videoio, to obtain images of the webcam or file
 * - opencv_imgcodecs, to get video file images
 * - opencv_imgproc
 * - opencv_calib3d
 * - GL
 * - GLU
 *
 * Eigen folder in Properties for os1, c / c ++ General Settings, Path and Symbols, Includes:
 * Usr / local / include / eigen3
 *
 * Eigen folder in Properties for os1, c / c ++ General Settings, Path and Symbols, Symbols, G ++:
 * - _cplusplus 201103L
 * - ArchiveBowBinary
 * - COMPILEDWITHC11
 *
 * \ Section desc Initial sequence of the ORB-SLAM2 algorithm
 * As usual, execution starts with main, which:
 * - read execution command line parameters
 * - creates the SLAM object, the only instance of the System class, whose constructor
 * - load the BoW vocabulary
 * - load the configuration file and parse it
 * - creates threads for the four main and parallel processes:
 * - LoopClosing
 * - LocalMapping
 * - Viewer
 * - Tracking (running on the main thread).
 * - enters the main loop while (true) that will execute the entire process for each frame of the camera
 *
 * This main loop is a long algorithm, separated into methods by neatness, most of which are invoked from a single place.
 * The sequence of invocations is:
 * 1 - System :: TrackMonocular, passing the color image of the camera
 * 2- Tracking :: GrabImageMonocular, passing the Mat image in grayscale, creates the currentFrame from the image
 * 3- Tracking :: Track, finite automaton that dispatches methods according to the state.
 * 4- Tracking :: TrackWithMotionModel, invoked in the OK state
 *
 * \ Section clas Classification of classes
 * ORB-SLAM2 has a .hy file and another .cc file for each class, all declared in the ORB-SLAM2 namespace.
 * Adopts the following suffixes to name class member properties, specifying their type:
* - m: member. All properties begin with m.
 * - p: pointer
 * - v: vector
 * - b: boolean
 * - n: integer
 * - f: float
 * There are exceptions, probably by mistake.
 *
 * Examples:
 * - mvpMapPoints is a pointer vector member.
 * - mnId is an integer member Id.
 *
 *
 * There are several types of class:
 * - Classes that are repeatedly instantiated:
 * - Frame
 * - KeyFrame
 * - MapPoint
 * - Initializer
 * - ORBextractor
 * - ORBmatcher
 * - Classes that are instantiated only once and are associated with a thread:
 * - Tracking
 * - LocalMapping
 * - Viewer
 * - LoopClosing
 * - Classes that are instantiated only once and last:
 * - System
 * - Map
 * - KeyFrameDatabase
 * - MapDrawer
 * - FrameDrawer
 * - Classes that are not instantiated, have no properties, are repositories of class methods:
 * - Converter
 * - Optimizer
 *
 * Exceptions:
 * ORBVocabulary.h is an exception to this rule, it does not define a class but a simple typedef.
 * ORBExtractor.h defines two classes, including ExtractorNode.
 * The Thirparty folder contains pruned versions of DBoW2 and g2o with their own styles.
 *
 *
 * \ Section conc ORB-SLAM concepts and their classes
 *
 * Frame:
 * An ephemeral Frame object is created from each camera image.
 * Usually the current frame and the previous frame only remain, with only two instances of this class being simultaneously in the system.
 * The Frame object has the singular points detected, its anti-distorted version, its descriptors and the associated map points.
 * The class provides the methods for its construction from the image.
 *
 * KeyFrame:
 * KeyFrame is created from a Frame when the system understands that it provides information to the map.
 * While a Frame is ephemeral, a KeyFrame is long-lived, it is the way a Frame becomes part of the map.
 * When a KeyFrame is created, it copies the main information from the current Frame, and computes more data, such as the BoWs of each descriptor.
 * The KeyFrame documentation explains the notation of arrays used in this and other classes.
 *
 * MapPoint:
* 3D point of the world map. It not only has the coordinates of the point, but also the list of KeyFrames that observe it,
 * The list of descriptors associated with the point (a descriptor for each KeyFrame that observes it), among others.
 *
 * Map:
* World map. It has a list of MapPoitns, a list of KeyFrames, and methods for map management.
 *
 *
 *
 * \ Section threads Description of threads
 * ORB-SLAM has four parallel threads, each with its own loop.
 *
 * - Tracking is the preponderant object of the main thread, which is responsible for processing each incoming image,
 * Detecting singular points, calculating descriptors, macheando with the previous frame
* Trying to identify known points on the map. Decide when to add a new Keyframe. Do not add new points to the map.
* - LocalMapping tries to add points to the map each time a KeyFrame is added. Add points by triangulation with neighboring KeyFrames.
 * Also optimizes the map by removing redundant keyframes.
 * - LoopClosing tries to compare the current observation with the map, looking for some pose that explains the current observation, thus detecting loops.
 * In this case proceed with the closing of the loop.
 * -Viewer handles the two views: the map and the processed image.
 *
 *
 */

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    void PublishPose(cv::Mat Tcw, ros::Time &stamp);

    ORB_SLAM2::System* mpSLAM;
    ros::Publisher* pPosPub;

};
void ImageGrabber::PublishPose(cv::Mat Tcw, ros::Time &stamp)// change
{
    geometry_msgs::PoseStamped poseMSG;
    if(!Tcw.empty())
    {

        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        Eigen::Matrix<double,3,3> rt=ORB_SLAM2::Converter::toMatrix3d(Rwc);
        Eigen::Vector3d eular_angular=rt.eulerAngles(2,1,0);
        cout<<"(x,y) "<<twc.at<float>(0)<<","<<twc.at<float>(2)<<"   pitch="<<eular_angular(1)*180/3.1415962<<endl;

        poseMSG.pose.position.x = twc.at<float>(2);
        poseMSG.pose.position.y = twc.at<float>(0);
        poseMSG.pose.position.z =0;
        
	Eigen::AngleAxisd rotationVector(eular_angular(1),Eigen::Vector3d(0,0,1));
        Eigen::Quaterniond q=Eigen::Quaterniond( rotationVector );
       
        poseMSG.pose.orientation.x = q.x();
        poseMSG.pose.orientation.y = q.y();
        poseMSG.pose.orientation.z = q.z();
        poseMSG.pose.orientation.w = q.w();
        poseMSG.header.frame_id = "VO";
        poseMSG.header.stamp = stamp;

        (pPosPub)->publish(poseMSG);

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
        if(argc != 2)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 settings_stereo" << endl;
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
    if (1 == save_map)
        save_map_bool=true;

    bool bReuseMap = false;
    if (1 == ReuseMap)
        bReuseMap = true;  
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true, bReuseMap);
    ORB_SLAM2::System SLAM(strORBvoc,strCamSet,ORB_SLAM2::System::RGBD,true,bReuseMap,strMapPath);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    
    ros::Publisher PosPub = nh.advertise<geometry_msgs::PoseStamped>("ORB_SLAM/pose", 5);//change
    
    igb.pPosPub = &(PosPub);//change
    ros::spin();
    // Stop all threads
    SLAM.Shutdown();
    if(save_map_bool)
		SLAM.SaveMapRequest();
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    
    if(ReuseMap)
         SLAM.Savenavigation("navigation.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw= mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
        ros::Time t=msgRGB->header.stamp;
     PublishPose(Tcw,t);
}


