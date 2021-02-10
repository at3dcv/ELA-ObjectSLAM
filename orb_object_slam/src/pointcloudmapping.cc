/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/*
 *--------------------------------------------------------------------------------------------------
 * DS-SLAM: A Semantic Visual SLAM towards Dynamic Environments
　*　Author(s):
 * Chao Yu, Zuxin Liu, Xinjun Liu, Fugui Xie, Yi Yang, Qi Wei, Fei Qiao qiaofei@mail.tsinghua.edu.cn
 * Created by Yu Chao@2018.12.03
 * --------------------------------------------------------------------------------------------------
 * DS-SLAM is a optimized SLAM system based on the famous ORB-SLAM2. If you haven't learn ORB_SLAM2 code, 
 * you'd better to be familiar with ORB_SLAM2 project first. Compared to ORB_SLAM2, 
 * we add anther two threads including semantic segmentation thread and densemap creation thread. 
 * You should pay attention to Frame.cc, ORBmatcher.cc, Pointcloudmapping.cc and Segment.cc.
 * 
 *　@article{murORB2,
 *　title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
　*　author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
　* journal={IEEE Transactions on Robotics},
　*　volume={33},
　* number={5},
　* pages={1255--1262},
　* doi = {10.1109/TRO.2017.2705103},
　* year={2017}
 *　}
 * --------------------------------------------------------------------------------------------------
 * Copyright (C) 2018, iVip Lab @ EE, THU (https://ivip-tsinghua.github.io/iViP-Homepage/) and 
 * Advanced Mechanism and Roboticized Equipment Lab. All rights reserved.
 *
 * Licensed under the GPLv3 License;
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * https://github.com/ivipsourcecode/DS-SLAM/blob/master/LICENSE
 *--------------------------------------------------------------------------------------------------
 */


#include "pointcloudmapping.h"
#include "Converter.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_broadcaster.h> 
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace ORB_SLAM2{


pcl::PointCloud<pcl::PointXYZRGBA> pcl_filter; 
ros::Publisher pclPoint_pub;
ros::Publisher octomap_pub;
sensor_msgs::PointCloud2 pcl_point;

pcl::PointCloud<pcl::PointXYZRGBA> pcl_cloud_kf;

PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    this->sor.setMeanK(50);                                
    this->sor.setStddevMulThresh(1.0);                    
    globalMap = boost::make_shared< PointCloud >( );
    KfMap = boost::make_shared< PointCloud >( );
    viewerThread = boost::make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );


}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);  
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& semantic_color,cv::Mat& semantic,cv::Mat& color, cv::Mat& depth)
{
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    semanticImgs.push_back(semantic.clone() );
    semanticImgs_color.push_back(semantic_color.clone() );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );

    keyFrameUpdated.notify_one();
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& semantic_color,cv::Mat& semantic, cv::Mat& color, cv::Mat& depth)
{
    float cam_w=480;
    float cam_h = 640;
    PointCloud::Ptr tmp( new PointCloud() );
    // Point cloud is null ptr
    for ( int m=0; m<depth.rows; m+=1)
    {
        for ( int n=0; n<depth.cols; n+=1)
        {
            float d = depth.ptr<float>(m)[n];

            //if (d < 0.01 || d > 8)
            if (d < 0 || d > 8)
                continue;
         
                PointT p;
                p.z = d;
                p.x = ( n - kf->cx) * p.z / kf->fx;
                p.y = ( m - kf->cy) * p.z / kf->fy;
	            // Deal with color

	            if((int)semantic.ptr<uchar>(m)[n]==0)
	            {
                    p.b = color.ptr<uchar>(m)[n*3];
                    p.g = color.ptr<uchar>(m)[n*3+1];
                    p.r = color.ptr<uchar>(m)[n*3+2];
	            }
	            else
	            {
                    p.b = semantic_color.ptr<uchar>(m)[n*3];
                    p.g = semantic_color.ptr<uchar>(m)[n*3+1];
                    p.r = semantic_color.ptr<uchar>(m)[n*3+2]; 
	            }
	            tmp->points.push_back(p);
               
        }
    }
    
    //Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );

    // use identity to see there is scale problem 
    // between the pose and depth map
    cv::Mat identity = cv::Mat::eye(4,4, CV_32F);
    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( identity ); 
    
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    cout<<"Generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    return cloud;
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr PointCloudMapping::generatePointCloudfromFeature(KeyFrame* kf, cv::Mat& semantic_color,cv::Mat& semantic, cv::Mat& color, cv::Mat& depth)
{
    std::vector<Eigen::Vector3f> box_colors;

	box_colors.push_back(Eigen::Vector3f(230, 0, 0) / 255.0);	 // red  0
	box_colors.push_back(Eigen::Vector3f(60, 180, 75) / 255.0);   // green  1
	box_colors.push_back(Eigen::Vector3f(0, 0, 255) / 255.0);	 // blue  2
	box_colors.push_back(Eigen::Vector3f(255, 0, 255) / 255.0);   // Magenta  3
	box_colors.push_back(Eigen::Vector3f(255, 165, 0) / 255.0);   // orange 4
	box_colors.push_back(Eigen::Vector3f(128, 0, 128) / 255.0);   //purple 5
	box_colors.push_back(Eigen::Vector3f(0, 255, 255) / 255.0);   //cyan 6
	box_colors.push_back(Eigen::Vector3f(210, 245, 60) / 255.0);  //lime  7
	box_colors.push_back(Eigen::Vector3f(250, 190, 190) / 255.0); //pink  8
	box_colors.push_back(Eigen::Vector3f(0, 128, 128) / 255.0);   //Teal  9
    //ROS_ERROR_STREAM("IN generatePointCloudfromFeature " <<  kf->N);
    PointCloud::Ptr tmp( new PointCloud() );

    // Set MapPoint vertices
    const int N = kf->N;

    {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++)
    {
        
        MapPoint *pMP = kf->mvpMapPoints[i];
        if (!pMP)
            continue;
        if (pMP->is_dynamic)
            continue;
        if (!pMP->mbTrackInView)
            continue;
        if (pMP->isBad())
            continue;

        float x = pMP->GetWorldPos().at<float>(0);
        float y = pMP->GetWorldPos().at<float>(1);
        float d = pMP->GetWorldPos().at<float>(2); 
        
        PointT p;
        // ROS_ERROR_STREAM(d << " " << x << " " << y);

        int xi = static_cast<int>(pMP->mTrackProjX);
        int yi = static_cast<int>(pMP->mTrackProjY);

        p.z = d;
        p.x = ( pMP->mTrackProjX - kf->cx) * p.z / kf->fx;
        p.y = ( pMP->mTrackProjY - kf->cy) * p.z / kf->fy;
        // Deal with color
        if((int)semantic.ptr<uchar>(xi)[yi]==0)
        {
            p.b = color.ptr<uchar>(xi)[yi*3];
            p.g = color.ptr<uchar>(xi)[yi*3+1];
            p.r = color.ptr<uchar>(xi)[yi*3+2];
        }
        else
        {
            p.b = 0;
            p.g = 0;
            p.r = 255;
        }
        tmp->points.push_back(p);
    
    }

    //landmarks of keyframe
    std::vector<MapObject *> cuboids_landmark = kf->cuboids_landmark;

    // Dye instances , employed from MapDrawer
    for (size_t object_id = 0; object_id < cuboids_landmark.size(); object_id++)
    {
        MapObject *obj_landmark = cuboids_landmark[object_id];
        if( !obj_landmark)
            continue;

        if (obj_landmark->isBad())
            continue;

        //owned_mappoints = obj_landmark->used_points_in_BA_filtered; // points really used in BA
        //if (whether_dynamic_object)
    
    	vector<MapPoint *> owned_mappoints = obj_landmark->GetUniqueMapPoints(); //map points that belongs to landmark object
        Eigen::Vector3f box_color = box_colors[obj_landmark->mnId % box_colors.size()];

        for (size_t pt_id = 0; pt_id < owned_mappoints.size(); pt_id++)
        {
            MapPoint *pMP = owned_mappoints[pt_id];
            if (!pMP)
                continue;
            if (pMP->is_dynamic)
                continue;
            if (!pMP->mbTrackInView)
                continue;
            if (pMP->isBad())
                continue;

            float x = pMP->GetWorldPos().at<float>(0);
            float y = pMP->GetWorldPos().at<float>(1);
            float d = pMP->GetWorldPos().at<float>(2); 
            PointT p;
            ROS_ERROR_STREAM(d << " " << x << " " << y);

            int xi = static_cast<int>(pMP->mTrackProjX);
            int yi = static_cast<int>(pMP->mTrackProjY);

            p.z = d;
            p.x = ( pMP->mTrackProjX - kf->cx) * p.z / kf->fx;
            p.y = ( pMP->mTrackProjY - kf->cy) * p.z / kf->fy;
            
            // Deal with color
        
            p.b = box_color.z();
            p.g = box_color.y();
            p.r = box_color.x();
            
            tmp->points.push_back(p);
        }
    }
    } 
    
    cv::Mat identity = cv::Mat::eye(4,4, CV_32F);

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( identity);
    
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    ROS_ERROR_STREAM("Generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size());

    return cloud;
    
}


void PointCloudMapping::viewer()
{
    ROS_ERROR_STREAM("IN VIEWER");

    ros::NodeHandle n;
    pclPoint_pub = n.advertise<sensor_msgs::PointCloud2>("/ORB_SLAM2_PointMap_SegNetM/Point_Clouds",100000);
    ros::Rate r(5);
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }
 
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }
        if(N==0)
	    {
	        cout<<"Keyframes miss!"<<endl;
            usleep(1000);
	        continue;
	    }
        KfMap->clear();
        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            //PointCloud::Ptr p = generatePointCloud( keyframes[i],semanticImgs_color[i], semanticImgs[i],colorImgs[i], depthImgs[i] );
            PointCloud::Ptr p = generatePointCloudfromFeature( keyframes[i],semanticImgs_color[i], semanticImgs[i],colorImgs[i], depthImgs[i] );

            *KfMap += *p;
	        *globalMap += *p;	    
        }
        

	    PointCloud::Ptr tmp1(new PointCloud());
        voxel.setInputCloud( KfMap );
        voxel.filter( *tmp1 );
        KfMap->swap( *tmp1 );	
        pcl_cloud_kf = *KfMap;	

	    Cloud_transform(pcl_cloud_kf,pcl_filter);
	    pcl::toROSMsg(pcl_filter, pcl_point);
	    pcl_point.header.frame_id = "/pointCloud";
	    pclPoint_pub.publish(pcl_point);

        lastKeyframeSize = N;
	    cout << "Keyframe map publish time ="<<endl;
    }

}

void PointCloudMapping::public_cloud( pcl::PointCloud< pcl::PointXYZRGBA >& cloud_kf )
{
	cloud_kf =pcl_cloud_kf; 
}

void PointCloudMapping::Cloud_transform(pcl::PointCloud<pcl::PointXYZRGBA>& source, pcl::PointCloud<pcl::PointXYZRGBA>& out)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered;
	Eigen::Matrix4f m;

	m<< 0,0,1,0,
	    -1,0,0,0,
		0,-1,0,0;
	Eigen::Affine3f transform(m);
	pcl::transformPointCloud (source, out, transform);
}
}