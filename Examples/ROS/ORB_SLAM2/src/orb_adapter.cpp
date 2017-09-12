/*
 * (c) 2017
 *
 * Authors:
 * Daniel Gehrig (ETH Zurich)
 * Maximilian Goettgens (ETH Zurich)
 * Brian Paden (MIT)
 *
 * This file was originally written as "ros_mono.cc" by Raul Mur-Artal and
 * has been modified by the authors to serve several tasks needed for the
 * ardrone to properly use the ORB SLAM 2 algorithm for state estimation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 * IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//////////////////////////////////TRANSFORMATIONS//////////////////////////////////////////////////////////////////
//To fuse the orb SLAM pose estimate with onboard sensor data, it is
//necessary to publish any other sensor data and the orb SLAM data in the same parent frame which is typically
//called 'odom'. See REP105 and REP103 on ros.org for further details on the concept.
//
//The final transformation for the orb SLAM looks like this:
//
//    odom --> second_keyframe_base_link --> second_keyframe_cam --> first_keyframe_cam
//    --> orb_pose_unscaled_cam --> orb_pose_unscaled_base_link
//
//Odom is defined by the drone on startup (this is not the startup time of the driver node but the time the plug is connected)
//Odom is defined with it's x-axis facing north and the negative z-axis aligned to the gravitation vector and must
//not be changed at any time afterwards.
//
//The first and second keyframe transformation is set once the orb slam initializes (meaning it is able to estimate a position
//for the first time). It is set to be the transformation from odom to ardrone_base_frontcam (published by the driver).
//It is important to mention, that the first transformation, that is actually received from the ORB SLAM algorithm is the
//transformation to the SECOND keyframe. There is a need to compensate for this to get a correct pose from the first keyframe on.
//
//Camera frames always come in a way such that the z axis is pointing forwards while the y axis is
//facing downwards. Therefore the transformations to switch frames are here achieved through a manual rotation such that eventually
//the orb pose values which are beeing publsihed to a PoseStamped topic represent the correct pose within the odom frame.
//See the rqt_tf_tree for further calrification on the single transformations this code is performing.
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include"../../../include/System.h"

#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <nav_msgs/Path.h>

#include <tf2/LinearMath/Transform.h>
#include <std_msgs/Float32.h>

#include<opencv2/core/core.hpp>

#include <System.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <ardrone_orb_adapter.h>

using namespace std;

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "Mono" );
    ros::start();

    if ( argc != 3 ) {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb ( &SLAM );

    stringstream ss(argv[3]);
    ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
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

    ros::NodeHandle nodeHandler;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nodeHandler, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nodeHandler, "camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));


    ros::Publisher pointcloud_pub = nodeHandler.advertise<sensor_msgs::PointCloud> ( "/orb/point_cloud", 2 );
    //ros::Publisher pose_unscaled_pub = nodeHandler.advertise<geometry_msgs::PoseStamped> ( "/orb/pose_unscaled", 2 );
    ros::Publisher pose_scaled_pub = nodeHandler.advertise<geometry_msgs::PoseStamped> ( "/orb/pose_scaled", 2 );
    ros::Publisher path_pub = nodeHandler.advertise<nav_msgs::Path> ( "/ardrone/path_scaled",2 );

    ros::Rate loop_rate ( 30 );
    int counter = 0;

    while ( ros::ok() ) {
        ros::spinOnce();
        if ( igb.pc_init_  && counter % 30 == 0 ) {
            pointcloud_pub.publish ( igb.pc_ ); //publish at 1 Hz
        }

        if ( igb.pose_init_ && igb.tracking_state_ == 2 ) {
            //pose_unscaled_pub.publish ( igb.pose_out_unscaled_ );
            pose_scaled_pub.publish ( igb.pose_out_scaled_ );
            if ( igb.isPathTracking_ ) {
                igb.updatePathArray();
                path_pub.publish ( igb.scaled_path_ );
            }
        }
        counter++;
        if (counter == 3000) counter = 0; //preventing overflow of int
        
        loop_rate.sleep();
    }

    // Stop all threads
    SLAM.Shutdown();

    ros::shutdown();

    return 0;
}

// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
