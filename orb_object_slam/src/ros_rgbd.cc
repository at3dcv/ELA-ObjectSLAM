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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "Parameters.h"
#include "tictoc_profiler/profiler.hpp"

// LL: Added config header to pass macro that switches Leander's code off and on
#include "At3dcv_config.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    ORB_SLAM2::System *mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    ca::Profiler::enable();

    if (argc != 3)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }
    ros::NodeHandle nh;

    bool enable_loop_closing = true;
    nh.param<bool>("enable_viewer", ORB_SLAM2::enable_viewer, true);
    nh.param<bool>("enable_viewmap", ORB_SLAM2::enable_viewmap, true);
    nh.param<bool>("enable_viewimage", ORB_SLAM2::enable_viewimage, true);
    nh.param<bool>("enable_loop_closing", enable_loop_closing, true);
    nh.param<bool>("parallel_mapping", ORB_SLAM2::parallel_mapping, true);

    nh.param<bool>("whether_detect_object", ORB_SLAM2::whether_detect_object, false);
    nh.param<bool>("whether_read_offline_cuboidtxt", ORB_SLAM2::whether_read_offline_cuboidtxt, false);
    nh.param<bool>("associate_point_with_object", ORB_SLAM2::associate_point_with_object, false);

    nh.param<bool>("whether_dynamic_object", ORB_SLAM2::whether_dynamic_object, false);
    nh.param<bool>("remove_dynamic_features", ORB_SLAM2::remove_dynamic_features, false);

    nh.param<bool>("mono_firstframe_truth_depth_init", ORB_SLAM2::mono_firstframe_truth_depth_init, false);
    nh.param<bool>("mono_firstframe_Obj_depth_init", ORB_SLAM2::mono_firstframe_Obj_depth_init, false);
    nh.param<bool>("mono_allframe_Obj_depth_init", ORB_SLAM2::mono_allframe_Obj_depth_init, false);

    nh.param<bool>("enable_ground_height_scale", ORB_SLAM2::enable_ground_height_scale, false);
    nh.param<bool>("use_dynamic_klt_features", ORB_SLAM2::use_dynamic_klt_features, false);

    nh.param<bool>("bundle_object_opti", ORB_SLAM2::bundle_object_opti, false);
    nh.param<double>("camera_object_BA_weight", ORB_SLAM2::camera_object_BA_weight, 1.0);
    nh.param<double>("object_velocity_BA_weight", ORB_SLAM2::object_velocity_BA_weight, 1.0);

    nh.param<bool>("draw_map_truth_paths", ORB_SLAM2::draw_map_truth_paths, true);
    nh.param<bool>("draw_nonlocal_mappoint", ORB_SLAM2::draw_nonlocal_mappoint, true);

    // temp debug
    nh.param<bool>("ba_dyna_pt_obj_cam", ORB_SLAM2::ba_dyna_pt_obj_cam, false);
    nh.param<bool>("ba_dyna_obj_velo", ORB_SLAM2::ba_dyna_obj_velo, true);
    nh.param<bool>("ba_dyna_obj_cam", ORB_SLAM2::ba_dyna_obj_cam, true);

    std::string scene_name;
    ros::param::get("/scene_name", scene_name);
    ros::param::get("/base_data_folder", ORB_SLAM2::base_data_folder);

    if (scene_name.compare(std::string("kitti")) == 0)
        ORB_SLAM2::scene_unique_id = ORB_SLAM2::kitti;

    cout << "Base_data_folder:  " << ORB_SLAM2::base_data_folder << endl;

    std::string packagePath = ros::package::getPath("orb_object_slam");

    if (!enable_loop_closing)
        ROS_WARN_STREAM("Turn off global loop closing!!");
    else
        ROS_WARN_STREAM("Turn on global loop closing!!");
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // AC: Set System to MONOCULAR as we don't leverage the depth data
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, enable_loop_closing);

    ImageGrabber igb(&SLAM);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    ros::spin(); //block here till I ctrl-C

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //     SLAM.SaveKeyFrameTrajectoryTUM(packagePath+"/Outputs/KeyFrameTrajectory.txt");
    // Save camera trajectory
    SLAM.SaveTrajectoryTUM(packagePath + "/Outputs/AllFrameTrajectory.txt");
    if (ORB_SLAM2::scene_unique_id == ORB_SLAM2::kitti)
        SLAM.SaveTrajectoryKITTI(packagePath + "/Outputs/AllFrameTrajectoryKITTI.txt");

    ca::Profiler::print_aggregated(std::cout);
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    cv::Mat depth_mat;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
        depth_mat = cv_ptrD->image;
        // usually TYPE_32FC1   if TYPE_16UC1, convert it.
        if (msgD->encoding == sensor_msgs::image_encodings::TYPE_16UC1) // already uint, (already multiplied by 1000)
        {
            depth_mat.convertTo(depth_mat, CV_32FC1, 1.0 / 1000.0, 0);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    
    #ifdef at3dcv_andy
    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image, depth_mat, cv_ptrRGB->header.stamp.toSec(), msgRGB->header.seq);
    #elif defined(at3dcv_tum_rgbd)
    std::string unix_file_name = unix_stamp_as_identifier(cv_ptrRGB->header.stamp);
    std::cout << "$$$$$$$$$$$$$$$$$$ unix_file_name " << unix_file_name<< std::endl;
    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image, depth_mat, cv_ptrRGB->header.stamp.toSec(), unix_file_name);
    #else
    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image, depth_mat, cv_ptrRGB->header.stamp.toSec());
    #endif
}
