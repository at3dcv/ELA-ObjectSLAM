// std c
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <ctime>
#include <algorithm>

// opencv pcl
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

// ours
#include "detect_3d_cuboid/matrix_utils.h"
#include "detect_3d_cuboid/object_3d_util.h"
#include "tictoc_profiler/profiler.hpp"

using namespace std;
// using namespace cv;
using namespace Eigen;

// LL: Copy the camera intrisic matrix and its invers
void detect_3d_cuboid::set_calibration(const Matrix3d &Kalib)
{
	cam_pose.Kalib = Kalib;
	cam_pose.invK = Kalib.inverse();
}

// LL: Setting up the camera's transformation matrices
void detect_3d_cuboid::set_cam_pose(const Matrix4d &transToWolrd)
{
	cam_pose.transToWolrd = transToWolrd;
	// LL: Matrix4d.topLeftCorner(3,3) returns the submatrix x_ij with i,j element 0-2
	cam_pose.rotationToWorld = transToWolrd.topLeftCorner<3, 3>();
	Vector3d euler_angles;
	// LL Transform the rotational quaternion to euler representation
	quat_to_euler_zyx(Quaterniond(cam_pose.rotationToWorld), euler_angles(0), euler_angles(1), euler_angles(2));
	cam_pose.euler_angle = euler_angles;
	cam_pose.invR = cam_pose.rotationToWorld.inverse();
	// LL: Calculate projection matrix K * [R|t]^-1 for the projection of the world coordinates to the camera coordinates
	// LL: We use the inverser as the argument passed to the function is transToWorld and we need transToCamera
	cam_pose.projectionMatrix = cam_pose.Kalib * transToWolrd.inverse().topRows<3>(); // project world coordinate to camera
	// LL: K * R^-1
	cam_pose.KinvR = cam_pose.Kalib * cam_pose.invR;
	cam_pose.camera_yaw = cam_pose.euler_angle(2);
	//TODO relative measure? not good... then need to change transToWolrd.
}

void detect_3d_cuboid::detect_cuboid(const cv::Mat &rgb_img, const Matrix4d &transToWolrd, const MatrixXd &obj_bbox_coors,
									 MatrixXd all_lines_raw, std::vector<ObjectSet> &all_object_cuboids)
{
	set_cam_pose(transToWolrd);
	cam_pose_raw = cam_pose;

	cv::Mat gray_img;
	if (rgb_img.channels() == 3)
		cv::cvtColor(rgb_img, gray_img, CV_BGR2GRAY);
	else
		gray_img = rgb_img;

	int img_width = rgb_img.cols;
	int img_height = rgb_img.rows;

	int num_2d_objs = obj_bbox_coors.rows();
	all_object_cuboids.resize(num_2d_objs);

	vector<bool> all_configs;
	all_configs.push_back(consider_config_1);
	all_configs.push_back(consider_config_2);

	// parameters for cuboid generation
	double vp12_edge_angle_thre = 15;
	double vp3_edge_angle_thre = 10;	// 10  10  parameters
	double shorted_edge_thre = 20;		// if box edge are too short. box might be too thin. most possibly wrong.
	bool reweight_edge_distance = true; // if want to compare with all configurations. we need to reweight

	// parameters for proposal scoring
	bool whether_normalize_two_errors = true;
	double weight_vp_angle = 0.8;
	double weight_skew_error = 1.5;
	// if also consider config2, need to weight two erros, in order to compare two configurations

	align_left_right_edges(all_lines_raw); // this should be guaranteed when detecting edges
	if (whether_plot_detail_images)
	{
		cv::Mat output_img;
		plot_image_with_edges(rgb_img, output_img, all_lines_raw, cv::Scalar(255, 0, 0));
		cv::imshow("Raw detected Edges", output_img); //cv::waitKey(0);
	}

	// find ground-wall boundary edges
	Vector4d ground_plane_world(0, 0, 1, 0); // treated as column vector % in my pop-up code, I use [0 0 -1 0]. here I want the normal pointing innerwards, towards the camera to match surface normal prediction
	Vector4d ground_plane_sensor = cam_pose.transToWolrd.transpose() * ground_plane_world;

	//       int object_id=1;
	for (int object_id = 0; object_id < num_2d_objs; object_id++)
	{
		// 	  std::cout<<"object id  "<<object_id<<std::endl;
		ca::Profiler::tictoc("One 3D object total time");
		int left_x_raw = obj_bbox_coors(object_id, 0);
		int top_y_raw = obj_bbox_coors(object_id, 1);
		int obj_width_raw = obj_bbox_coors(object_id, 2);
		int obj_height_raw = obj_bbox_coors(object_id, 3);
		int right_x_raw = left_x_raw + obj_bbox_coors(object_id, 2);
		int down_y_raw = top_y_raw + obj_height_raw;

		std::vector<int> down_expand_sample_all;
		down_expand_sample_all.push_back(0);
		if (whether_sample_bbox_height) // 2D object detection might not be accurate
		{
			int down_expand_sample_ranges = max(min(20, obj_height_raw - 90), 20);
			down_expand_sample_ranges = min(down_expand_sample_ranges, img_height - top_y_raw - obj_height_raw - 1); // should lie inside the image  -1 for c++ index
			if (down_expand_sample_ranges > 10)																		 // if expand large margin, give more samples.
				down_expand_sample_all.push_back(round(down_expand_sample_ranges / 2));
			down_expand_sample_all.push_back(down_expand_sample_ranges);
		}

		// NOTE later if in video, could use previous object yaw..., also reduce search range
		double yaw_init = cam_pose.camera_yaw - 90.0 / 180.0 * M_PI; // yaw init is directly facing the camera, align with camera optical axis
		std::vector<double> obj_yaw_samples;
		linespace<double>(yaw_init - 45.0 / 180.0 * M_PI, yaw_init + 45.0 / 180.0 * M_PI, 6.0 / 180.0 * M_PI, obj_yaw_samples);

		MatrixXd all_configs_errors(400, 9);
		MatrixXd all_box_corners_2ds(800, 8);   // initialize a large eigen matrix
		int valid_config_number_all_height = 0; // all valid objects of all height samples
		ObjectSet raw_obj_proposals;
		raw_obj_proposals.reserve(100);
		// 	    int sample_down_expan_id=1;
		for (int sample_down_expan_id = 0; sample_down_expan_id < down_expand_sample_all.size(); sample_down_expan_id++)
		{
			int down_expand_sample = down_expand_sample_all[sample_down_expan_id];
			int obj_height_expan = obj_height_raw + down_expand_sample;
			int down_y_expan = top_y_raw + obj_height_expan;
			double obj_diaglength_expan = sqrt(obj_width_raw * obj_width_raw + obj_height_expan * obj_height_expan);

			// sample points on the top edges, if edge is too large, give more samples. give at least 10 samples for all edges. for small object, object pose changes lots
			int top_sample_resolution = round(min(20, obj_width_raw / 10)); //  25 pixels
			std::vector<int> top_x_samples;
			linespace<int>(left_x_raw + 5, right_x_raw - 5, top_sample_resolution, top_x_samples);
			MatrixXd sample_top_pts(2, top_x_samples.size());
			for (int ii = 0; ii < top_x_samples.size(); ii++)
			{
				sample_top_pts(0, ii) = top_x_samples[ii];
				sample_top_pts(1, ii) = top_y_raw;
			}

			// expand some small margin for distance map  [10 20]
			int distmap_expand_wid = min(max(min(20, obj_width_raw - 100), 10), max(min(20, obj_height_expan - 100), 10));
			int left_x_expan_distmap = max(0, left_x_raw - distmap_expand_wid);
			int right_x_expan_distmap = min(img_width - 1, right_x_raw + distmap_expand_wid);
			int top_y_expan_distmap = max(0, top_y_raw - distmap_expand_wid);
			int down_y_expan_distmap = min(img_height - 1, down_y_expan + distmap_expand_wid);
			int height_expan_distmap = down_y_expan_distmap - top_y_expan_distmap;
			int width_expan_distmap = right_x_expan_distmap - left_x_expan_distmap;
			Vector2d expan_distmap_lefttop = Vector2d(left_x_expan_distmap, top_y_expan_distmap);
			Vector2d expan_distmap_rightbottom = Vector2d(right_x_expan_distmap, down_y_expan_distmap);

			// find edges inside the object bounding box
			MatrixXd all_lines_inside_object(all_lines_raw.rows(), all_lines_raw.cols()); // first allocate a large matrix, then only use the toprows to avoid copy, alloc
			int inside_obj_edge_num = 0;
			for (int edge_id = 0; edge_id < all_lines_raw.rows(); edge_id++)
				if (check_inside_box(all_lines_raw.row(edge_id).head<2>(), expan_distmap_lefttop, expan_distmap_rightbottom))
					if (check_inside_box(all_lines_raw.row(edge_id).tail<2>(), expan_distmap_lefttop, expan_distmap_rightbottom))
					{
						all_lines_inside_object.row(inside_obj_edge_num) = all_lines_raw.row(edge_id);
						inside_obj_edge_num++;
					}

			// merge edges and remove short lines, after finding object edges.  edge merge in small regions should be faster than all.
			double pre_merge_dist_thre = 20;
			double pre_merge_angle_thre = 5;
			double edge_length_threshold = 30;
			MatrixXd all_lines_merge_inobj;
			merge_break_lines(all_lines_inside_object.topRows(inside_obj_edge_num), all_lines_merge_inobj, pre_merge_dist_thre,
							  pre_merge_angle_thre, edge_length_threshold);

			// compute edge angels and middle points
			VectorXd lines_inobj_angles(all_lines_merge_inobj.rows());
			MatrixXd edge_mid_pts(all_lines_merge_inobj.rows(), 2);
			for (int i = 0; i < all_lines_merge_inobj.rows(); i++)
			{
				lines_inobj_angles(i) = std::atan2(all_lines_merge_inobj(i, 3) - all_lines_merge_inobj(i, 1), all_lines_merge_inobj(i, 2) - all_lines_merge_inobj(i, 0)); // [-pi/2 -pi/2]
				edge_mid_pts.row(i).head<2>() = (all_lines_merge_inobj.row(i).head<2>() + all_lines_merge_inobj.row(i).tail<2>()) / 2;
			}

			// TODO could canny or distance map outside sampling height to speed up!!!!   Then only need to compute canny onces.
			// detect canny edges and compute distance transform  NOTE opencv canny maybe different from matlab. but roughly same
			cv::Rect object_bbox = cv::Rect(left_x_expan_distmap, top_y_expan_distmap, width_expan_distmap, height_expan_distmap); //
			cv::Mat im_canny;
			cv::Canny(gray_img(object_bbox), im_canny, 80, 200); // low thre, high thre    im_canny 0 or 255   [80 200  40 100]
			cv::Mat dist_map;
			cv::distanceTransform(255 - im_canny, dist_map, CV_DIST_L2, 3); // dist_map is float datatype

			if (whether_plot_detail_images)
			{
				cv::imshow("im_canny", im_canny);
				cv::Mat dist_map_img;
				cv::normalize(dist_map, dist_map_img, 0.0, 1.0, cv::NORM_MINMAX);
				cv::imshow("normalized distance map", dist_map_img);
				cv::waitKey();
			}

			// Generate cuboids
			MatrixXd all_configs_error_one_objH(200, 9);
			MatrixXd all_box_corners_2d_one_objH(400, 8);
			int valid_config_number_one_objH = 0;

			std::vector<double> cam_roll_samples;
			std::vector<double> cam_pitch_samples;
			if (whether_sample_cam_roll_pitch)
			{
				linespace<double>(cam_pose_raw.euler_angle(0) - 6.0 / 180.0 * M_PI, cam_pose_raw.euler_angle(0) + 6.0 / 180.0 * M_PI, 3.0 / 180.0 * M_PI, cam_roll_samples);
				linespace<double>(cam_pose_raw.euler_angle(1) - 6.0 / 180.0 * M_PI, cam_pose_raw.euler_angle(1) + 6.0 / 180.0 * M_PI, 3.0 / 180.0 * M_PI, cam_pitch_samples);
			}
			else
			{
				cam_roll_samples.push_back(cam_pose_raw.euler_angle(0));
				cam_pitch_samples.push_back(cam_pose_raw.euler_angle(1));
			}
			// different from matlab. first for loop yaw, then for configurations.
			// 	      int obj_yaw_id=8;
			for (int cam_roll_id = 0; cam_roll_id < cam_roll_samples.size(); cam_roll_id++)
				for (int cam_pitch_id = 0; cam_pitch_id < cam_pitch_samples.size(); cam_pitch_id++)
					for (int obj_yaw_id = 0; obj_yaw_id < obj_yaw_samples.size(); obj_yaw_id++)
					{
						if (whether_sample_cam_roll_pitch)
						{
							Matrix4d transToWolrd_new = transToWolrd;
							transToWolrd_new.topLeftCorner<3, 3>() = euler_zyx_to_rot<double>(cam_roll_samples[cam_roll_id], cam_pitch_samples[cam_pitch_id], cam_pose_raw.euler_angle(2));
							set_cam_pose(transToWolrd_new);
							ground_plane_sensor = cam_pose.transToWolrd.transpose() * ground_plane_world;
						}

						double obj_yaw_esti = obj_yaw_samples[obj_yaw_id];

						Vector2d vp_1, vp_2, vp_3;
						getVanishingPoints(cam_pose.KinvR, obj_yaw_esti, vp_1, vp_2, vp_3); // for object x y z  axis

						MatrixXd all_vps(3, 2);
						all_vps.row(0) = vp_1;
						all_vps.row(1) = vp_2;
						all_vps.row(2) = vp_3;
						// 		  std::cout<<"obj_yaw_esti  "<<obj_yaw_esti<<"  "<<obj_yaw_id<<std::endl;
						MatrixXd all_vp_bound_edge_angles = VP_support_edge_infos(all_vps, edge_mid_pts, lines_inobj_angles,
																				  Vector2d(vp12_edge_angle_thre, vp3_edge_angle_thre));
						// 		  int sample_top_pt_id=15;
						for (int sample_top_pt_id = 0; sample_top_pt_id < sample_top_pts.cols(); sample_top_pt_id++)
						{
							// 		      std::cout<<"sample_top_pt_id "<<sample_top_pt_id<<std::endl;
							Vector2d corner_1_top = sample_top_pts.col(sample_top_pt_id);
							bool config_good = true;
							int vp_1_position = 0; // 0 initial as fail,  1  on left   2 on right
							Vector2d corner_2_top = seg_hit_boundary(vp_1, corner_1_top, Vector4d(right_x_raw, top_y_raw, right_x_raw, down_y_expan));
							if (corner_2_top(0) == -1)
							{ // vp1-corner1 doesn't hit the right boundary. check whether hit left
								corner_2_top = seg_hit_boundary(vp_1, corner_1_top, Vector4d(left_x_raw, top_y_raw, left_x_raw, down_y_expan));
								if (corner_2_top(0) != -1) // vp1-corner1 hit the left boundary   vp1 on the right
									vp_1_position = 2;
							}
							else // vp1-corner1 hit the right boundary   vp1 on the left
								vp_1_position = 1;

							config_good = vp_1_position > 0;
							if (!config_good)
							{
								if (print_details)
									printf("Configuration fails at corner 2, outside segment\n");
								continue;
							}
							if ((corner_1_top - corner_2_top).norm() < shorted_edge_thre)
							{
								if (print_details)
									printf("Configuration fails at edge 1-2, too short\n");
								continue;
							}
							// 		      cout<<"corner_1/2   "<<corner_1_top.transpose()<<"   "<<corner_2_top.transpose()<<endl;
							// 		      int config_ind=0; // have to consider config now.
							for (int config_id = 1; config_id < 3; config_id++) // configuration one or two of matlab version
							{
								if (!all_configs[config_id - 1])
									continue;
								Vector2d corner_3_top, corner_4_top;
								if (config_id == 1)
								{
									if (vp_1_position == 1) // then vp2 hit the left boundary
										corner_4_top = seg_hit_boundary(vp_2, corner_1_top, Vector4d(left_x_raw, top_y_raw, left_x_raw, down_y_expan));
									else // or, then vp2 hit the right boundary
										corner_4_top = seg_hit_boundary(vp_2, corner_1_top, Vector4d(right_x_raw, top_y_raw, right_x_raw, down_y_expan));
									if (corner_4_top(1) == -1)
									{
										config_good = false;
										if (print_details)
											printf("Configuration %d fails at corner 4, outside segment\n", config_id);
										continue;
									}
									if ((corner_1_top - corner_4_top).norm() < shorted_edge_thre)
									{
										if (print_details)
											printf("Configuration %d fails at edge 1-4, too short\n", config_id);
										continue;
									}
									// compute the last point in the top face
									corner_3_top = lineSegmentIntersect(vp_2, corner_2_top, vp_1, corner_4_top, true);
									if (!check_inside_box(corner_3_top, Vector2d(left_x_raw, top_y_raw), Vector2d(right_x_raw, down_y_expan)))
									{ // check inside boundary. otherwise edge visibility might be wrong
										config_good = false;
										if (print_details)
											printf("Configuration %d fails at corner 3, outside box\n", config_id);
										continue;
									}
									if (((corner_3_top - corner_4_top).norm() < shorted_edge_thre) || ((corner_3_top - corner_2_top).norm() < shorted_edge_thre))
									{
										if (print_details)
											printf("Configuration %d fails at edge 3-4/3-2, too short\n", config_id);
										continue;
									}
									// 			      cout<<"corner_3/4   "<<corner_3_top.transpose()<<"   "<<corner_4_top.transpose()<<endl;
								}
								if (config_id == 2)
								{
									if (vp_1_position == 1) // then vp2 hit the left boundary
										corner_3_top = seg_hit_boundary(vp_2, corner_2_top, Vector4d(left_x_raw, top_y_raw, left_x_raw, down_y_expan));
									else // or, then vp2 hit the right boundary
										corner_3_top = seg_hit_boundary(vp_2, corner_2_top, Vector4d(right_x_raw, top_y_raw, right_x_raw, down_y_expan));
									if (corner_3_top(1) == -1)
									{
										config_good = false;
										if (print_details)
											printf("Configuration %d fails at corner 3, outside segment\n", config_id);
										continue;
									}
									if ((corner_2_top - corner_3_top).norm() < shorted_edge_thre)
									{
										if (print_details)
											printf("Configuration %d fails at edge 2-3, too short\n", config_id);
										continue;
									}
									// compute the last point in the top face
									corner_4_top = lineSegmentIntersect(vp_1, corner_3_top, vp_2, corner_1_top, true);
									if (!check_inside_box(corner_4_top, Vector2d(left_x_raw, top_y_expan_distmap), Vector2d(right_x_raw, down_y_expan_distmap)))
									{
										config_good = false;
										if (print_details)
											printf("Configuration %d fails at corner 4, outside box\n", config_id);
										continue;
									}
									if (((corner_3_top - corner_4_top).norm() < shorted_edge_thre) || ((corner_4_top - corner_1_top).norm() < shorted_edge_thre))
									{
										if (print_details)
											printf("Configuration %d fails at edge 3-4/4-1, too short\n", config_id);
										continue;
									}
									// 			      cout<<"corner_3/4   "<<corner_3_top.transpose()<<"   "<<corner_4_top.transpose()<<endl;
								}
								// compute first bottom points    computing bottom points is the same for config 1,2
								Vector2d corner_5_down = seg_hit_boundary(vp_3, corner_3_top, Vector4d(left_x_raw, down_y_expan, right_x_raw, down_y_expan));
								if (corner_5_down(1) == -1)
								{
									config_good = false;
									if (print_details)
										printf("Configuration %d fails at corner 5, outside segment\n", config_id);
									continue;
								}
								if ((corner_3_top - corner_5_down).norm() < shorted_edge_thre)
								{
									if (print_details)
										printf("Configuration %d fails at edge 3-5, too short\n", config_id);
									continue;
								}
								Vector2d corner_6_down = lineSegmentIntersect(vp_2, corner_5_down, vp_3, corner_2_top, true);
								if (!check_inside_box(corner_6_down, expan_distmap_lefttop, expan_distmap_rightbottom))
								{
									config_good = false;
									if (print_details)
										printf("Configuration %d fails at corner 6, outside box\n", config_id);
									continue;
								}
								if (((corner_6_down - corner_2_top).norm() < shorted_edge_thre) || ((corner_6_down - corner_5_down).norm() < shorted_edge_thre))
								{
									if (print_details)
										printf("Configuration %d fails at edge 6-5/6-2, too short\n", config_id);
									continue;
								}
								Vector2d corner_7_down = lineSegmentIntersect(vp_1, corner_6_down, vp_3, corner_1_top, true);
								if (!check_inside_box(corner_7_down, expan_distmap_lefttop, expan_distmap_rightbottom))
								{ // might be slightly different from matlab
									config_good = false;
									if (print_details)
										printf("Configuration %d fails at corner 7, outside box\n", config_id);
									continue;
								}
								if (((corner_7_down - corner_1_top).norm() < shorted_edge_thre) || ((corner_7_down - corner_6_down).norm() < shorted_edge_thre))
								{
									if (print_details)
										printf("Configuration %d fails at edge 7-1/7-6, too short\n", config_id);
									continue;
								}
								Vector2d corner_8_down = lineSegmentIntersect(vp_1, corner_5_down, vp_2, corner_7_down, true);
								if (!check_inside_box(corner_8_down, expan_distmap_lefttop, expan_distmap_rightbottom))
								{
									config_good = false;
									if (print_details)
										printf("Configuration %d fails at corner 8, outside box\n", config_id);
									continue;
								}
								if (((corner_8_down - corner_4_top).norm() < shorted_edge_thre) || ((corner_8_down - corner_5_down).norm() < shorted_edge_thre) || ((corner_8_down - corner_7_down).norm() < shorted_edge_thre))
								{
									if (print_details)
										printf("Configuration %d fails at edge 8-4/8-5/8-7, too short\n", config_id);
									continue;
								}

								MatrixXd box_corners_2d_float(2, 8);
								box_corners_2d_float << corner_1_top, corner_2_top, corner_3_top, corner_4_top, corner_5_down, corner_6_down, corner_7_down, corner_8_down;
								// 			  std::cout<<"box_corners_2d_float \n "<<box_corners_2d_float<<std::endl;
								MatrixXd box_corners_2d_float_shift(2, 8);
								box_corners_2d_float_shift.row(0) = box_corners_2d_float.row(0).array() - left_x_expan_distmap;
								box_corners_2d_float_shift.row(1) = box_corners_2d_float.row(1).array() - top_y_expan_distmap;

								MatrixXi visible_edge_pt_ids, vps_box_edge_pt_ids;
								double sum_dist;
								if (config_id == 1)
								{
									visible_edge_pt_ids.resize(9, 2);
									visible_edge_pt_ids << 1, 2, 2, 3, 3, 4, 4, 1, 2, 6, 3, 5, 4, 8, 5, 8, 5, 6;
									vps_box_edge_pt_ids.resize(3, 4);
									vps_box_edge_pt_ids << 1, 2, 8, 5, 4, 1, 5, 6, 4, 8, 2, 6; // six edges. each row represents two edges [e1_1 e1_2   e2_1 e2_2;...] of one VP
									visible_edge_pt_ids.array() -= 1;
									vps_box_edge_pt_ids.array() -= 1; //change to c++ index
									sum_dist = box_edge_sum_dists(dist_map, box_corners_2d_float_shift, visible_edge_pt_ids);
								}
								else
								{
									visible_edge_pt_ids.resize(7, 2);
									visible_edge_pt_ids << 1, 2, 2, 3, 3, 4, 4, 1, 2, 6, 3, 5, 5, 6;
									vps_box_edge_pt_ids.resize(3, 4);
									vps_box_edge_pt_ids << 1, 2, 3, 4, 4, 1, 5, 6, 3, 5, 2, 6; // six edges. each row represents two edges [e1_1 e1_2   e2_1 e2_2;...] of one VP
									visible_edge_pt_ids.array() -= 1;
									vps_box_edge_pt_ids.array() -= 1;
									sum_dist = box_edge_sum_dists(dist_map, box_corners_2d_float_shift, visible_edge_pt_ids, reweight_edge_distance);
								}
								double total_angle_diff = box_edge_alignment_angle_error(all_vp_bound_edge_angles, vps_box_edge_pt_ids, box_corners_2d_float);
								all_configs_error_one_objH.row(valid_config_number_one_objH).head<4>() = Vector4d(config_id, vp_1_position, obj_yaw_esti, sample_top_pt_id);
								all_configs_error_one_objH.row(valid_config_number_one_objH).segment<3>(4) = Vector3d(sum_dist / obj_diaglength_expan, total_angle_diff, down_expand_sample);
								if (whether_sample_cam_roll_pitch)
									all_configs_error_one_objH.row(valid_config_number_one_objH).segment<2>(7) = Vector2d(cam_roll_samples[cam_roll_id], cam_pitch_samples[cam_pitch_id]);
								else
									all_configs_error_one_objH.row(valid_config_number_one_objH).segment<2>(7) = Vector2d(cam_pose_raw.euler_angle(0), cam_pose_raw.euler_angle(1));
								all_box_corners_2d_one_objH.block(2 * valid_config_number_one_objH, 0, 2, 8) = box_corners_2d_float;
								valid_config_number_one_objH++;
								if (valid_config_number_one_objH >= all_configs_error_one_objH.rows())
								{
									all_configs_error_one_objH.conservativeResize(2 * valid_config_number_one_objH, NoChange);
									all_box_corners_2d_one_objH.conservativeResize(4 * valid_config_number_one_objH, NoChange);
								}
							} //end of config loop
						}	 //end of top id
					}		  //end of yaw

			// 	      std::cout<<"valid_config_number_one_hseight  "<<valid_config_number_one_objH<<std::endl;
			// 	      std::cout<<"all_configs_error_one_objH  \n"<<all_configs_error_one_objH.topRows(valid_config_number_one_objH)<<std::endl;
			// 	      MatrixXd all_corners = all_box_corners_2d_one_objH.topRows(2*valid_config_number_one_objH);
			// 	      std::cout<<"all corners   "<<all_corners<<std::endl;

			VectorXd normalized_score;
			vector<int> good_proposal_ids;
			fuse_normalize_scores_v2(all_configs_error_one_objH.col(4).head(valid_config_number_one_objH), all_configs_error_one_objH.col(5).head(valid_config_number_one_objH),
									 normalized_score, good_proposal_ids, weight_vp_angle, whether_normalize_two_errors);

			for (int box_id = 0; box_id < good_proposal_ids.size(); box_id++)
			{
				int raw_cube_ind = good_proposal_ids[box_id];

				if (whether_sample_cam_roll_pitch)
				{
					Matrix4d transToWolrd_new = transToWolrd;
					transToWolrd_new.topLeftCorner<3, 3>() = euler_zyx_to_rot<double>(all_configs_error_one_objH(raw_cube_ind, 7), all_configs_error_one_objH(raw_cube_ind, 8), cam_pose_raw.euler_angle(2));
					set_cam_pose(transToWolrd_new);
					ground_plane_sensor = cam_pose.transToWolrd.transpose() * ground_plane_world;
				}

				cuboid *sample_obj = new cuboid();
				change_2d_corner_to_3d_object(all_box_corners_2d_one_objH.block(2 * raw_cube_ind, 0, 2, 8), all_configs_error_one_objH.row(raw_cube_ind).head<3>(),
											  ground_plane_sensor, cam_pose.transToWolrd, cam_pose.invK, cam_pose.projectionMatrix, *sample_obj);
				// 		  sample_obj->print_cuboid();
				if ((sample_obj->scale.array() < 0).any())
					continue; // scale should be positive
				sample_obj->rect_detect_2d = Vector4d(left_x_raw, top_y_raw, obj_width_raw, obj_height_raw);
				sample_obj->edge_distance_error = all_configs_error_one_objH(raw_cube_ind, 4); // record the original error
				sample_obj->edge_angle_error = all_configs_error_one_objH(raw_cube_ind, 5);
				sample_obj->normalized_error = normalized_score(box_id);
				double skew_ratio = sample_obj->scale.head(2).maxCoeff() / sample_obj->scale.head(2).minCoeff();
				sample_obj->skew_ratio = skew_ratio;
				sample_obj->down_expand_height = all_configs_error_one_objH(raw_cube_ind, 6);
				if (whether_sample_cam_roll_pitch)
				{
					sample_obj->camera_roll_delta = all_configs_error_one_objH(raw_cube_ind, 7) - cam_pose_raw.euler_angle(0);
					sample_obj->camera_pitch_delta = all_configs_error_one_objH(raw_cube_ind, 8) - cam_pose_raw.euler_angle(1);
				}
				else
				{
					sample_obj->camera_roll_delta = 0;
					sample_obj->camera_pitch_delta = 0;
				}

				raw_obj_proposals.push_back(sample_obj);
			}
		} // end of differnet object height sampling

		// %finally rank all proposals. [normalized_error   skew_error]
		int actual_cuboid_num_small = std::min(max_cuboid_num, (int)raw_obj_proposals.size());
		VectorXd all_combined_score(raw_obj_proposals.size());
		for (int box_id = 0; box_id < raw_obj_proposals.size(); box_id++)
		{
			cuboid *sample_obj = raw_obj_proposals[box_id];
			double skew_error = weight_skew_error * std::max(sample_obj->skew_ratio - nominal_skew_ratio, 0.0);
			if (sample_obj->skew_ratio > max_cut_skew)
				skew_error = 100;
			double new_combined_error = sample_obj->normalized_error + weight_skew_error * skew_error;
			all_combined_score(box_id) = new_combined_error;
		}

		std::vector<int> sort_idx_small(all_combined_score.rows());
		iota(sort_idx_small.begin(), sort_idx_small.end(), 0);
		sort_indexes(all_combined_score, sort_idx_small, actual_cuboid_num_small);
		for (int ii = 0; ii < actual_cuboid_num_small; ii++) // use sorted index
		{
			all_object_cuboids[object_id].push_back(raw_obj_proposals[sort_idx_small[ii]]);
		}

		ca::Profiler::tictoc("One 3D object total time");
	} // end of different objects

	if (whether_plot_final_images || whether_save_final_images)
	{
		cv::Mat frame_all_cubes_img = rgb_img.clone();
		for (int object_id = 0; object_id < all_object_cuboids.size(); object_id++)
			if (all_object_cuboids[object_id].size() > 0)
			{
				plot_image_with_cuboid(frame_all_cubes_img, all_object_cuboids[object_id][0]);
			}
		if (whether_save_final_images)
			cuboids_2d_img = frame_all_cubes_img;
		if (whether_plot_final_images)
		{
			cv::imshow("frame_all_cubes_img", frame_all_cubes_img);
			cv::waitKey(0);
		}
	}
}

// LL: Added by Leander
#ifdef at3dcv_leander
// LL: Added by Leander: Overloaded function by adding `read_inst_segment_vert` and `yolo_obj_class`
void detect_3d_cuboid::detect_cuboid(const cv::Mat &rgb_img, const Matrix4d &transToWolrd, const MatrixXd &obj_bbox_coors, MatrixXd all_lines_raw, 
									std::vector<ObjectSet> &all_object_cuboids, std::vector<Eigen::Matrix2Xd> read_inst_segment_vert, std::vector<std::string> yolo_obj_class)
{
	/* Args:
	* 		rgb_img: Raw RGB image
	*		transToworld: 4*4 camera pose
	*		obj_bbox_coors: Matrix of the 2d BB, each row holds the four corners of a single bounding box; BB's too close to boundary where allready removed
	*		all_lines_raw: Lines in the current frame detected by the LSD line detector
	*		all_object_cuboids: same as `obj_bbox_coors` but as a vector of of 1x4 row matrices.
	*		read_inst_segment_vert: Vector containing n*(2*i) matrices representing the vertices of the instance segmentation masks.
	*								n = number of object detected in the current frame, 2 = x and y value of the vertix, i = vertices of the inst. seg. mask 
	*		yolo_obj_class: The class names of the detected objects in the current frame 
	*/

	// LL: Added by Leander: Ensure both vectors (vertices and cuboids) have the same length (same number of objects).
	if ((int)all_object_cuboids.size() != (int)read_inst_segment_vert.size() && (int)yolo_obj_class.size() != (int)read_inst_segment_vert.size())
	{
		std::cout << "all_object_cuboids.size" << (int)all_object_cuboids.size() << std::endl;
		std::cout << "read_inst_segment_vert.size" << (int)read_inst_segment_vert.size() << std::endl;
		std::cout << "yolo_obj_class.size" << (int)yolo_obj_class.size() << std::endl;
		ROS_ERROR_STREAM("The number of cuboids, instance segmentation masks and class names does not match");
	}

	// LL: Calibrate the camera using the provided transformation matrix for camera to world trans.
	set_cam_pose(transToWolrd);
	cam_pose_raw = cam_pose;

	// LL: Convert the image to gray image
	cv::Mat gray_img;
	if (rgb_img.channels() == 3)
		cv::cvtColor(rgb_img, gray_img, CV_BGR2GRAY);
	else
		gray_img = rgb_img;

	// LL: Retrive the width and the height of the image
	int img_width = rgb_img.cols;
	int img_height = rgb_img.rows;

	// LL: Retrive the actual number of 2D BB's and resize 
	int num_2d_objs = obj_bbox_coors.rows();
	all_object_cuboids.resize(num_2d_objs);

	// LL: Retrive configuration`s (important mode parameters for proposal generation), both are set to true in detect_3d_cuboid.h
	vector<bool> all_configs;
	all_configs.push_back(consider_config_1);
	all_configs.push_back(consider_config_2);

	// LL: Hardcoded tresholds for the cuboid proposels
	// parameters for cuboid generation
	double vp12_edge_angle_thre = 15;
	double vp3_edge_angle_thre = 10;	// 10  10  parameters
	double shorted_edge_thre = 20;		// if box edge are too short. box might be too thin. most possibly wrong.
	bool reweight_edge_distance = true; // if want to compare with all configurations. we need to reweight

	// LL: Weights for how how strongly the different aspects are weighted for the cuboid proposels
	// parameters for proposal scoring
	bool whether_normalize_two_errors = true;
	double weight_vp_angle = 0.8;
	double weight_skew_error = 1.5;
	// if also consider config2, need to weight two erros, in order to compare two configurations

	// LL: Make sure edges start from left to right (x_1 < x_2)
	align_left_right_edges(all_lines_raw); // this should be guaranteed when detecting edges

	// LL: Plots image if whether_plot_detail_images=true
	if (whether_plot_detail_images)
	{
		cv::Mat output_img;
		plot_image_with_edges(rgb_img, output_img, all_lines_raw, cv::Scalar(255, 0, 0));
		cv::imshow("Raw detected Edges", output_img); //cv::waitKey(0);
	}

	// LL: Define the ground plane in camera and world coordinates
	// find ground-wall boundary edges
	Vector4d ground_plane_world(0, 0, 1, 0); // treated as column vector % in my pop-up code, I use [0 0 -1 0]. here I want the normal pointing innerwards, towards the camera to match surface normal prediction
	Vector4d ground_plane_sensor = cam_pose.transToWolrd.transpose() * ground_plane_world;

	// LL: Loop over all 2D BB, generate cuboid proposels, score them and pick the one with highest score 
	for (int object_id = 0; object_id < num_2d_objs; object_id++)
	{
		ca::Profiler::tictoc("One 3D object total time");

		// LL: Retrive the relevant information from the object bbox cores for object "object_id"
		int left_x_raw = obj_bbox_coors(object_id, 0);
		int top_y_raw = obj_bbox_coors(object_id, 1);
		int obj_width_raw = obj_bbox_coors(object_id, 2);
		int obj_height_raw = obj_bbox_coors(object_id, 3);
		int right_x_raw = left_x_raw + obj_bbox_coors(object_id, 2);
		int down_y_raw = top_y_raw + obj_height_raw;

		// LL: Vector that is supposed to hold the box hight calculated in the if case below
		// LL: Since `whether_sample_bbox_height` is by default set to false down_expand_sample_all simple equals [0]
		std::vector<int> down_expand_sample_all;
		down_expand_sample_all.push_back(0);

		// LL: Calculate the bounding box hight based on raw detection (inancurate -> default value for whether_sample_bbox_height=false)
		if (whether_sample_bbox_height) // 2D object detection might not be accurate
		{
			int down_expand_sample_ranges = max(min(20, obj_height_raw - 90), 20);
			down_expand_sample_ranges = min(down_expand_sample_ranges, img_height - top_y_raw - obj_height_raw - 1); // should lie inside the image  -1 for c++ index
			if (down_expand_sample_ranges > 10)																		 // if expand large margin, give more samples.
				down_expand_sample_all.push_back(round(down_expand_sample_ranges / 2));
			down_expand_sample_all.push_back(down_expand_sample_ranges);
		}

		// LL: Create search space for the yaw of the current object
		// LL: Should be improved, bad style
		// NOTE later if in video, could use previous object yaw..., also reduce search range
		double yaw_init = cam_pose.camera_yaw - 90.0 / 180.0 * M_PI; // yaw init is directly facing the camera, align with camera optical axis
		std::vector<double> obj_yaw_samples;
		linespace<double>(yaw_init - 45.0 / 180.0 * M_PI, yaw_init + 45.0 / 180.0 * M_PI, 6.0 / 180.0 * M_PI, obj_yaw_samples);

		// LL: Matrix to hold the calculated errors of all the cuboid proposels for the current object
		MatrixXd all_configs_errors(400, 9);

		// LL: Initialize a vector of cuboids -> typedef std::vector<cuboid *> ObjectSet in detect_3d_cuboid.h
		ObjectSet raw_obj_proposals;
		raw_obj_proposals.reserve(100);

		// LL: Loop over the calculated bounding box higths
		// LL: Since `whether_sample_bbox_height` above is set to false  `down_expand_sample_all` = [0] -> only a single iteration is run
		for (int sample_down_expan_id = 0; sample_down_expan_id < down_expand_sample_all.size(); sample_down_expan_id++)
		{
			// LL: For the case that `whether_sample_bbox_height` = false (default case), the for loop is only run for a single iteration with `down_expand_sample_all[sample_down_expan_id]` = 0
			// LL: Calculate BB width and diagonal
			int down_expand_sample = down_expand_sample_all[sample_down_expan_id];
			int obj_height_expan = obj_height_raw + down_expand_sample;
			int down_y_expan = top_y_raw + obj_height_expan;
			double obj_diaglength_expan = sqrt(obj_width_raw * obj_width_raw + obj_height_expan * obj_height_expan);

			// LL: Samples points along the top edge of the BB, i.e. along the line (left_x_raw, top_y_raw) and (right_x_raw, top_y_raw)
			// sample points on the top edges, if edge is too large, give more samples. give at least 10 samples for all edges. for small object, object pose changes lots
			int top_sample_resolution = round(min(20, obj_width_raw / 10)); //  25 pixels
			std::vector<int> top_x_samples;
			linespace<int>(left_x_raw + 5, right_x_raw - 5, top_sample_resolution, top_x_samples);
			// LL: `sample_top_pts` holds the sampled point along the top BB
			MatrixXd sample_top_pts(2, top_x_samples.size());
			for (int ii = 0; ii < top_x_samples.size(); ii++)
			{
				sample_top_pts(0, ii) = top_x_samples[ii];
				sample_top_pts(1, ii) = top_y_raw;
			}

			// LL: Create by 10 to 20 pixels inflated BB version
			// expand some small margin for distance map  [10 20]
			int distmap_expand_wid = min(max(min(20, obj_width_raw - 100), 10), max(min(20, obj_height_expan - 100), 10));
			int left_x_expan_distmap = max(0, left_x_raw - distmap_expand_wid);
			int right_x_expan_distmap = min(img_width - 1, right_x_raw + distmap_expand_wid);
			int top_y_expan_distmap = max(0, top_y_raw - distmap_expand_wid);
			int down_y_expan_distmap = min(img_height - 1, down_y_expan + distmap_expand_wid);
			int height_expan_distmap = down_y_expan_distmap - top_y_expan_distmap;
			int width_expan_distmap = right_x_expan_distmap - left_x_expan_distmap;
			Vector2d expan_distmap_lefttop = Vector2d(left_x_expan_distmap, top_y_expan_distmap);
			Vector2d expan_distmap_rightbottom = Vector2d(right_x_expan_distmap, down_y_expan_distmap);

			// LL: Check which lines lie inside the 2D BB by first checking if (x_1,y_1) is inside and then checking if (x_2, y_2) is inside for each line
			// LL: Lines that are inside the BB are stored in `all_lines_inside_object`
			// find edges inside the object bounding box
			MatrixXd all_lines_inside_object(all_lines_raw.rows(), all_lines_raw.cols()); // first allocate a large matrix, then only use the toprows to avoid copy, alloc
			int inside_obj_edge_num = 0;
			for (int edge_id = 0; edge_id < all_lines_raw.rows(); edge_id++)
				if (check_inside_box(all_lines_raw.row(edge_id).head<2>(), expan_distmap_lefttop, expan_distmap_rightbottom))
					if (check_inside_box(all_lines_raw.row(edge_id).tail<2>(), expan_distmap_lefttop, expan_distmap_rightbottom))
					{
						all_lines_inside_object.row(inside_obj_edge_num) = all_lines_raw.row(edge_id);
						inside_obj_edge_num++;
					}

			// LL: Filter out edges that are to short and merge edges in close proximity 
			// merge edges and remove short lines, after finding object edges.  edge merge in small regions should be faster than all.
			double pre_merge_dist_thre = 20;
			double pre_merge_angle_thre = 5;
			double edge_length_threshold = 30;
			MatrixXd all_lines_merge_inobj;
			merge_break_lines(all_lines_inside_object.topRows(inside_obj_edge_num), all_lines_merge_inobj, pre_merge_dist_thre,
							  pre_merge_angle_thre, edge_length_threshold);

			// compute edge angels and middle points
			VectorXd lines_inobj_angles(all_lines_merge_inobj.rows());
			MatrixXd edge_mid_pts(all_lines_merge_inobj.rows(), 2);
			for (int i = 0; i < all_lines_merge_inobj.rows(); i++)
			{
				lines_inobj_angles(i) = std::atan2(all_lines_merge_inobj(i, 3) - all_lines_merge_inobj(i, 1), all_lines_merge_inobj(i, 2) - all_lines_merge_inobj(i, 0)); // [-pi/2 -pi/2]
				edge_mid_pts.row(i).head<2>() = (all_lines_merge_inobj.row(i).head<2>() + all_lines_merge_inobj.row(i).tail<2>()) / 2;
			}

			// LL: Create gray image depicting the expanded 2D BB
			// LL: Apply the canny edge detector to the image to receive an "edge detected" version of the image
			// LL: Calculate the distance transform "in order to obtain the derived representation of a binary image, where the value of each pixel is replaced by its distance to the nearest background pixel"
			// TODO could canny or distance map outside sampling height to speed up!!!!   Then only need to compute canny onces.
			// detect canny edges and compute distance transform  NOTE opencv canny maybe different from matlab. but roughly same
			cv::Rect object_bbox = cv::Rect(left_x_expan_distmap, top_y_expan_distmap, width_expan_distmap, height_expan_distmap); //
			cv::Mat im_canny;
			cv::Canny(gray_img(object_bbox), im_canny, 80, 200); // low thre, high thre    im_canny 0 or 255   [80 200  40 100]
			cv::Mat dist_map;
			cv::distanceTransform(255 - im_canny, dist_map, CV_DIST_L2, 3); // dist_map is float datatype

			if (whether_plot_detail_images)
			{
				cv::imshow("im_canny", im_canny);
				cv::Mat dist_map_img;
				cv::normalize(dist_map, dist_map_img, 0.0, 1.0, cv::NORM_MINMAX);
				cv::imshow("normalized distance map", dist_map_img);
				cv::waitKey();
			}

			// LL: Matrix that stores all configurations and errors related to cuboid proposels.
			// Generate cuboids
			MatrixXd all_configs_error_one_objH(200, 9);
			// LL: Matrix that stores the eight 2D calculated/sampled corners of the cuboid proposels
			MatrixXd all_box_corners_2d_one_objH(400, 8);
			int valid_config_number_one_objH = 0;

			// LL: `whether_sample_cam_roll_pitch` default is false.
			// LL: If set to true init sampling spaces for raw an pitch 
			std::vector<double> cam_roll_samples;
			std::vector<double> cam_pitch_samples;
			if (whether_sample_cam_roll_pitch)
			{
				linespace<double>(cam_pose_raw.euler_angle(0) - 6.0 / 180.0 * M_PI, cam_pose_raw.euler_angle(0) + 6.0 / 180.0 * M_PI, 3.0 / 180.0 * M_PI, cam_roll_samples);
				linespace<double>(cam_pose_raw.euler_angle(1) - 6.0 / 180.0 * M_PI, cam_pose_raw.euler_angle(1) + 6.0 / 180.0 * M_PI, 3.0 / 180.0 * M_PI, cam_pitch_samples);
			}
			else
			{
				cam_roll_samples.push_back(cam_pose_raw.euler_angle(0));
				cam_pitch_samples.push_back(cam_pose_raw.euler_angle(1));
			}

			// LL: Since `whether_sample_cam_roll_pitch` is set to false outer and first inner for loop only do a single iteration.
			// LL: Iterates through the prior defined yaw search space for the single given roll and pitch value.
			for (int cam_roll_id = 0; cam_roll_id < cam_roll_samples.size(); cam_roll_id++)
				for (int cam_pitch_id = 0; cam_pitch_id < cam_pitch_samples.size(); cam_pitch_id++)
					for (int obj_yaw_id = 0; obj_yaw_id < obj_yaw_samples.size(); obj_yaw_id++)
					{
						if (whether_sample_cam_roll_pitch)
						{
							Matrix4d transToWolrd_new = transToWolrd;
							transToWolrd_new.topLeftCorner<3, 3>() = euler_zyx_to_rot<double>(cam_roll_samples[cam_roll_id], cam_pitch_samples[cam_pitch_id], cam_pose_raw.euler_angle(2));
							set_cam_pose(transToWolrd_new);
							ground_plane_sensor = cam_pose.transToWolrd.transpose() * ground_plane_world;
						}

						double obj_yaw_esti = obj_yaw_samples[obj_yaw_id];

						// LL: Calculate the vanishing points
						Vector2d vp_1, vp_2, vp_3;
						getVanishingPoints(cam_pose.KinvR, obj_yaw_esti, vp_1, vp_2, vp_3); // for object x y z  axis

						MatrixXd all_vps(3, 2);
						all_vps.row(0) = vp_1;
						all_vps.row(1) = vp_2;
						all_vps.row(2) = vp_3;
						
						// LL: Calculate the edges and there angles bounding the vanishing points
						MatrixXd all_vp_bound_edge_angles = VP_support_edge_infos(all_vps, edge_mid_pts, lines_inobj_angles,
																				  Vector2d(vp12_edge_angle_thre, vp3_edge_angle_thre));

						// LL: Loop over all the points that we previously sampled along the top edge of the BB
						for (int sample_top_pt_id = 0; sample_top_pt_id < sample_top_pts.cols(); sample_top_pt_id++)
						{
							// LL: Sample top corner one
							Vector2d corner_1_top = sample_top_pts.col(sample_top_pt_id);
							bool config_good = true;
							int vp_1_position = 0; // 0 initial as fail,  1  on left   2 on right
							// LL: Calculate top corner two
							Vector2d corner_2_top = seg_hit_boundary(vp_1, corner_1_top, Vector4d(right_x_raw, top_y_raw, right_x_raw, down_y_expan));
							// LL: Test if calculated top corner forfills requirements.
							// LL: If not: Take new sample for corner one, calculate corner two, and check again 
							if (corner_2_top(0) == -1)
							{ // vp1-corner1 doesn't hit the right boundary. check whether hit left
								corner_2_top = seg_hit_boundary(vp_1, corner_1_top, Vector4d(left_x_raw, top_y_raw, left_x_raw, down_y_expan));
								if (corner_2_top(0) != -1) // vp1-corner1 hit the left boundary   vp1 on the right
									vp_1_position = 2;
							}
							else // vp1-corner1 hit the right boundary   vp1 on the left
								vp_1_position = 1;

							config_good = vp_1_position > 0;
							if (!config_good)
							{
								if (print_details)
									printf("Configuration fails at corner 2, outside segment\n");
								continue;
							}
							if ((corner_1_top - corner_2_top).norm() < shorted_edge_thre)
							{
								if (print_details)
									printf("Configuration fails at edge 1-2, too short\n");
								continue;
							}

							// LL: Loop through two different "configs"/methods to calculate top corners three and four.
							for (int config_id = 1; config_id < 3; config_id++) // configuration one or two of matlab version
							{
								if (!all_configs[config_id - 1])
									continue;
								Vector2d corner_3_top, corner_4_top;
								
								// LL: Calculate the top corner three and four using method one
								if (config_id == 1)
								{
									if (vp_1_position == 1) // then vp2 hit the left boundary
										corner_4_top = seg_hit_boundary(vp_2, corner_1_top, Vector4d(left_x_raw, top_y_raw, left_x_raw, down_y_expan));
									else // or, then vp2 hit the right boundary
										corner_4_top = seg_hit_boundary(vp_2, corner_1_top, Vector4d(right_x_raw, top_y_raw, right_x_raw, down_y_expan));
									if (corner_4_top(1) == -1)
									{
										config_good = false;
										if (print_details)
											printf("Configuration %d fails at corner 4, outside segment\n", config_id);
										continue;
									}
									if ((corner_1_top - corner_4_top).norm() < shorted_edge_thre)
									{
										if (print_details)
											printf("Configuration %d fails at edge 1-4, too short\n", config_id);
										continue;
									}
									// compute the last point in the top face
									corner_3_top = lineSegmentIntersect(vp_2, corner_2_top, vp_1, corner_4_top, true);
									if (!check_inside_box(corner_3_top, Vector2d(left_x_raw, top_y_raw), Vector2d(right_x_raw, down_y_expan)))
									{ // check inside boundary. otherwise edge visibility might be wrong
										config_good = false;
										if (print_details)
											printf("Configuration %d fails at corner 3, outside box\n", config_id);
										continue;
									}
									if (((corner_3_top - corner_4_top).norm() < shorted_edge_thre) || ((corner_3_top - corner_2_top).norm() < shorted_edge_thre))
									{
										if (print_details)
											printf("Configuration %d fails at edge 3-4/3-2, too short\n", config_id);
										continue;
									}
								}
								// LL: Calculate the top corner three and four using method one
								if (config_id == 2)
								{
									if (vp_1_position == 1) // then vp2 hit the left boundary
										corner_3_top = seg_hit_boundary(vp_2, corner_2_top, Vector4d(left_x_raw, top_y_raw, left_x_raw, down_y_expan));
									else // or, then vp2 hit the right boundary
										corner_3_top = seg_hit_boundary(vp_2, corner_2_top, Vector4d(right_x_raw, top_y_raw, right_x_raw, down_y_expan));
									if (corner_3_top(1) == -1)
									{
										config_good = false;
										if (print_details)
											printf("Configuration %d fails at corner 3, outside segment\n", config_id);
										continue;
									}
									if ((corner_2_top - corner_3_top).norm() < shorted_edge_thre)
									{
										if (print_details)
											printf("Configuration %d fails at edge 2-3, too short\n", config_id);
										continue;
									}
									// compute the last point in the top face
									corner_4_top = lineSegmentIntersect(vp_1, corner_3_top, vp_2, corner_1_top, true);
									if (!check_inside_box(corner_4_top, Vector2d(left_x_raw, top_y_expan_distmap), Vector2d(right_x_raw, down_y_expan_distmap)))
									{
										config_good = false;
										if (print_details)
											printf("Configuration %d fails at corner 4, outside box\n", config_id);
										continue;
									}
									if (((corner_3_top - corner_4_top).norm() < shorted_edge_thre) || ((corner_4_top - corner_1_top).norm() < shorted_edge_thre))
									{
										if (print_details)
											printf("Configuration %d fails at edge 3-4/4-1, too short\n", config_id);
										continue;
									}
								}
								// LL: Calculate the bottom points
								// compute first bottom points computing bottom points is the same for config 1,2
								Vector2d corner_5_down = seg_hit_boundary(vp_3, corner_3_top, Vector4d(left_x_raw, down_y_expan, right_x_raw, down_y_expan));
								if (corner_5_down(1) == -1)
								{
									config_good = false;
									if (print_details)
										printf("Configuration %d fails at corner 5, outside segment\n", config_id);
									continue;
								}
								if ((corner_3_top - corner_5_down).norm() < shorted_edge_thre)
								{
									if (print_details)
										printf("Configuration %d fails at edge 3-5, too short\n", config_id);
									continue;
								}
								Vector2d corner_6_down = lineSegmentIntersect(vp_2, corner_5_down, vp_3, corner_2_top, true);
								if (!check_inside_box(corner_6_down, expan_distmap_lefttop, expan_distmap_rightbottom))
								{
									config_good = false;
									if (print_details)
										printf("Configuration %d fails at corner 6, outside box\n", config_id);
									continue;
								}
								if (((corner_6_down - corner_2_top).norm() < shorted_edge_thre) || ((corner_6_down - corner_5_down).norm() < shorted_edge_thre))
								{
									if (print_details)
										printf("Configuration %d fails at edge 6-5/6-2, too short\n", config_id);
									continue;
								}
								Vector2d corner_7_down = lineSegmentIntersect(vp_1, corner_6_down, vp_3, corner_1_top, true);
								if (!check_inside_box(corner_7_down, expan_distmap_lefttop, expan_distmap_rightbottom))
								{ // might be slightly different from matlab
									config_good = false;
									if (print_details)
										printf("Configuration %d fails at corner 7, outside box\n", config_id);
									continue;
								}
								if (((corner_7_down - corner_1_top).norm() < shorted_edge_thre) || ((corner_7_down - corner_6_down).norm() < shorted_edge_thre))
								{
									if (print_details)
										printf("Configuration %d fails at edge 7-1/7-6, too short\n", config_id);
									continue;
								}
								Vector2d corner_8_down = lineSegmentIntersect(vp_1, corner_5_down, vp_2, corner_7_down, true);
								if (!check_inside_box(corner_8_down, expan_distmap_lefttop, expan_distmap_rightbottom))
								{
									config_good = false;
									if (print_details)
										printf("Configuration %d fails at corner 8, outside box\n", config_id);
									continue;
								}
								if (((corner_8_down - corner_4_top).norm() < shorted_edge_thre) || ((corner_8_down - corner_5_down).norm() < shorted_edge_thre) || ((corner_8_down - corner_7_down).norm() < shorted_edge_thre))
								{
									if (print_details)
										printf("Configuration %d fails at edge 8-4/8-5/8-7, too short\n", config_id);
									continue;
								}

								// LL: Write all the calculated and sampled corners (2D) of the cuboid proposels to a 2x8 Eigen matrix
								MatrixXd box_corners_2d_float(2, 8);
								box_corners_2d_float << corner_1_top, corner_2_top, corner_3_top, corner_4_top, corner_5_down, corner_6_down, corner_7_down, corner_8_down;

								// LL: Shift the detected corners by calculated values. The value us either zero or small (10-20 pixels)
								MatrixXd box_corners_2d_float_shift(2, 8);
								box_corners_2d_float_shift.row(0) = box_corners_2d_float.row(0).array() - left_x_expan_distmap;
								box_corners_2d_float_shift.row(1) = box_corners_2d_float.row(1).array() - top_y_expan_distmap;

								// LL: Calculate the distance error between the edges of the eight 2D points of the cuboid propsel and the expended BB
								// LL: To different method depending on which config is currently set in the surronding for loop.
								MatrixXi visible_edge_pt_ids, vps_box_edge_pt_ids;
								double sum_dist;
								if (config_id == 1)
								{
									visible_edge_pt_ids.resize(9, 2);
									visible_edge_pt_ids << 1, 2, 2, 3, 3, 4, 4, 1, 2, 6, 3, 5, 4, 8, 5, 8, 5, 6;
									vps_box_edge_pt_ids.resize(3, 4);
									vps_box_edge_pt_ids << 1, 2, 8, 5, 4, 1, 5, 6, 4, 8, 2, 6; // six edges. each row represents two edges [e1_1 e1_2   e2_1 e2_2;...] of one VP
									visible_edge_pt_ids.array() -= 1;
									vps_box_edge_pt_ids.array() -= 1; //change to c++ index
									sum_dist = box_edge_sum_dists(dist_map, box_corners_2d_float_shift, visible_edge_pt_ids);
								}
								else
								{
									visible_edge_pt_ids.resize(7, 2);
									visible_edge_pt_ids << 1, 2, 2, 3, 3, 4, 4, 1, 2, 6, 3, 5, 5, 6;
									vps_box_edge_pt_ids.resize(3, 4);
									vps_box_edge_pt_ids << 1, 2, 3, 4, 4, 1, 5, 6, 3, 5, 2, 6; // six edges. each row represents two edges [e1_1 e1_2   e2_1 e2_2;...] of one VP
									visible_edge_pt_ids.array() -= 1;
									vps_box_edge_pt_ids.array() -= 1;
									sum_dist = box_edge_sum_dists(dist_map, box_corners_2d_float_shift, visible_edge_pt_ids, reweight_edge_distance);
								}
								// LL: Calculate the angle alignment error
								double total_angle_diff = box_edge_alignment_angle_error(all_vp_bound_edge_angles, vps_box_edge_pt_ids, box_corners_2d_float);
								
								// LL: Populate the config&error matrix with the calculated error values and all other relevant information
								// LL: The different proposels are mainly based on the sampled value along the top edge of the BB as well as the corresponding calculated vanishing points
								all_configs_error_one_objH.row(valid_config_number_one_objH).head<4>() = Vector4d(config_id, vp_1_position, obj_yaw_esti, sample_top_pt_id);
								all_configs_error_one_objH.row(valid_config_number_one_objH).segment<3>(4) = Vector3d(sum_dist / obj_diaglength_expan, total_angle_diff, down_expand_sample);
								
								// LL: Add the roll and pitch values to the config&error matrix
								if (whether_sample_cam_roll_pitch)
									all_configs_error_one_objH.row(valid_config_number_one_objH).segment<2>(7) = Vector2d(cam_roll_samples[cam_roll_id], cam_pitch_samples[cam_pitch_id]);
								else
									all_configs_error_one_objH.row(valid_config_number_one_objH).segment<2>(7) = Vector2d(cam_pose_raw.euler_angle(0), cam_pose_raw.euler_angle(1));
								
								// LL: Populate the 2D cuboid proposal matrix with the eight calculated/detected corners
								all_box_corners_2d_one_objH.block(2 * valid_config_number_one_objH, 0, 2, 8) = box_corners_2d_float;
								
								valid_config_number_one_objH++;

								// LL: Resize the matrices
								if (valid_config_number_one_objH >= all_configs_error_one_objH.rows())
								{
									all_configs_error_one_objH.conservativeResize(2 * valid_config_number_one_objH, NoChange);
									all_box_corners_2d_one_objH.conservativeResize(4 * valid_config_number_one_objH, NoChange);
								}
							} //end of config loop
						}	 //end of top id
					}		  //end of yaw

			VectorXd normalized_score;
			vector<int> good_proposal_ids;
			// LL: The function takes the distance error, angle error, weight factor for the angle error and a boolean whether to normalize or not (default is normalize).
			// LL: The errors are fused based on the function argument values and written to normalized_score
			// LL: The output is a vector of a limited number of the smallest values derived through following calculation: (dist_error + weight_vp_angle * angle_error) / (1 + weight_vp_angle)
			// LL: with the distance error and the angle error normalized.
			fuse_normalize_scores_v2(all_configs_error_one_objH.col(4).head(valid_config_number_one_objH), all_configs_error_one_objH.col(5).head(valid_config_number_one_objH),
									 normalized_score, good_proposal_ids, weight_vp_angle, whether_normalize_two_errors);

			// LL: Iterate over all proposels with a small enough normalized score.
			for (int box_id = 0; box_id < good_proposal_ids.size(); box_id++)
			{
				int raw_cube_ind = good_proposal_ids[box_id];

				// LL: In case that the roll and pitch got sampled use the information to update the transformation matrix for cam->world.
				if (whether_sample_cam_roll_pitch)
				{
					Matrix4d transToWolrd_new = transToWolrd;
					transToWolrd_new.topLeftCorner<3, 3>() = euler_zyx_to_rot<double>(all_configs_error_one_objH(raw_cube_ind, 7), all_configs_error_one_objH(raw_cube_ind, 8), cam_pose_raw.euler_angle(2));
					set_cam_pose(transToWolrd_new);
					ground_plane_sensor = cam_pose.transToWolrd.transpose() * ground_plane_world;
				}

				// LL: Added by Leander
				// Incase that no depth data is provided 
				#ifdef at3dcv_leander_depth
	
				//LL: Retrive the current 2d bb proposal
				//LL: Order the vectors to be: top_front_left, top_front_...
				//LL: Convert vertices to int
				Eigen::VectorXi cuboid_to_raw_boxstructIds(8);
				double vp_1_position = all_configs_error_one_objH.row(raw_cube_ind)(1);

				if (vp_1_position == 1) // vp1 on left, for all configurations
				    //cuboid_to_raw_boxstructIds << 6, 5, 8, 7, 2, 3, 4, 1;
					cuboid_to_raw_boxstructIds << 1, 4, 3, 2, 7, 8, 5, 6;
				if (vp_1_position == 2) // vp1 on right, for all configurations
				    //cuboid_to_raw_boxstructIds << 5, 6, 7, 8, 3, 2, 1, 4;
				    cuboid_to_raw_boxstructIds << 4, 1, 2, 3, 8, 7, 6, 5;

				Eigen::MatrixXi cub_prop_2d(2, 8);
				Eigen::MatrixXi cub_prop_2d_int(2, 8);
				cub_prop_2d_int = all_box_corners_2d_one_objH.block(2 * raw_cube_ind, 0, 2, 8).cast<int>();
				for (int i = 0; i < 8; i++)
				    cub_prop_2d.col(i) = cub_prop_2d_int.col(cuboid_to_raw_boxstructIds(i) - 1); // minius one to match index
				

				// LL: Split the current 2d BB in to six rectangles
				Eigen::MatrixXi sqr_1(2, 4);
				Eigen::MatrixXi sqr_2(2, 4);
				Eigen::MatrixXi sqr_3(2, 4);
				Eigen::MatrixXi sqr_4(2, 4);
				Eigen::MatrixXi sqr_5(2, 4);
				Eigen::MatrixXi sqr_6(2, 4); 

				sqr_1 << cub_prop_2d.col(0), cub_prop_2d.col(1), cub_prop_2d.col(2), cub_prop_2d.col(3);
				sqr_2 << cub_prop_2d.col(0), cub_prop_2d.col(1), cub_prop_2d.col(5), cub_prop_2d.col(4);
				sqr_3 << cub_prop_2d.col(0), cub_prop_2d.col(3), cub_prop_2d.col(7), cub_prop_2d.col(4);
				sqr_4 << cub_prop_2d.col(4), cub_prop_2d.col(5), cub_prop_2d.col(6), cub_prop_2d.col(7);
				sqr_5 << cub_prop_2d.col(6), cub_prop_2d.col(7), cub_prop_2d.col(3), cub_prop_2d.col(2);
				sqr_6 << cub_prop_2d.col(1), cub_prop_2d.col(2), cub_prop_2d.col(6), cub_prop_2d.col(5);

				std::vector<Eigen::MatrixXi> cub_surfaces;
				cub_surfaces.insert(cub_surfaces.end(),{sqr_1, sqr_2, sqr_3, sqr_4, sqr_5, sqr_6});

				// LL: Convert the six eigen matrix representations of the rectangles to boost styled strings
				std::vector<std::string> geometries;
				poly_vec_eigen_to_string_rep(cub_surfaces, geometries);

				// LL: Convert the boost styled strings to boost polygons and ensure the polygons are valid
				std::vector<polygon> surfaces;
				std::vector<std::string> colors;
				for (int i = 0; i != geometries.size(); ++i)
				{
				    colors.push_back("("+std::to_string(int(40)*i)+","+std::to_string(int(40)*i)+","+std::to_string(int(40)*i)+")");
				    polygon poly;
				    poly_string_to_boost_pooly(geometries[i], poly);
				    surfaces.push_back(poly);
				}

				// LL: Retrive the convex hull of the objects segmentation mask and convert the vertices to int
				std::cout << read_inst_segment_vert[object_id] << std::endl;
				//Eigen::Matrix<int, 2, read_inst_segment_vert[object_id].cols()> seg_mask_conv_hull;
				//seg_mask_conv_hull = read_inst_segment_vert[object_id].cast<int>();
				//std::cout << seg_mask_conv_hull << std::endl;
				// LL: Derive the boost polygon representation of the convexhull
				std::string segmentation_geometry;
				polygon poly_seg_mask_conv_hull;
				std::cout << "#### poly_string_to_boost_pooly ####" << std::endl;
				poly_eigen_to_string_rep(read_inst_segment_vert[object_id].cast<int>(), segmentation_geometry);
				poly_string_to_boost_pooly(segmentation_geometry, poly_seg_mask_conv_hull);
				std::cout << "#### poly_string_to_boost_pooly ####" << std::endl;

				// LL: Add a new color as well as the polygon to the collections
				colors.push_back("(140,140,140)");
				surfaces.push_back(poly_seg_mask_conv_hull);
				
				// LL: Document the text boost styled polygon representations
				std::string poly_file = "/mnt/datasets/output/polygons/"+std::to_string(object_id)+"_"+std::to_string(raw_cube_ind)+"_poly";
				std::ofstream out(poly_file+".txt");
				for(int i = 0; i != surfaces.size(); ++i)
					out << boost::geometry::wkt(surfaces[i]) << "\n";
				out.close();

				// LL: Plot the polygons and write the plot to a file of name <poly_file>.svg 
				visualize_polygons(poly_file, surfaces, colors);
				
				double percent_covered = 0.0;
				for (int i = 0; i != surfaces.size()-1; ++i)
				{
				    // LL: Calculate how much percent of the are of poly2 is covered by poly1
				    percent_covered += perc_poly2_covered_by_poly1(surfaces[i], surfaces[surfaces.size()-1]);
				}
				// LL: Do to geometrical constrains the six distinct sides of a rectangle can cover the surface of 
				// an object at most twice. => percent_covered/2 is a element of [0,1].
				percent_covered = percent_covered/2;
				std::cout << "#### percent_covered ####" << std::endl;
				std::cout << percent_covered << std::endl;
				std::cout << "#### percent_covered ####" << std::endl;
				
				// LL: Update the costfunction
				double weight_fac_cv_hull = 0.2;
				normalized_score(box_id) = (normalized_score(box_id)+ (weight_fac_cv_hull * percent_covered))/(1+weight_fac_cv_hull);
				#endif
				// LL: Added by Leander

				cuboid *sample_obj = new cuboid();


				change_2d_corner_to_3d_object(all_box_corners_2d_one_objH.block(2 * raw_cube_ind, 0, 2, 8), all_configs_error_one_objH.row(raw_cube_ind).head<3>(),
											  ground_plane_sensor, cam_pose.transToWolrd, cam_pose.invK, cam_pose.projectionMatrix, *sample_obj);
				// sample_obj->print_cuboid();
				if ((sample_obj->scale.array() < 0).any())
					continue; // scale should be positive
					
				// LL: Added by Leander
				// - Check if class name is in dic.
				// -- If not: Take default value estimated scale
				// -- If yes: Read value from dic
				std::unordered_map<std::string, Eigen::Vector3d>::iterator iter;  
				iter = sample_obj->obj_class_scales.find(yolo_obj_class[object_id]);

				if(iter != sample_obj->obj_class_scales.end())
				{
				    sample_obj->yolo_obj_scale = iter->second;
					std::cout << sample_obj->yolo_obj_scale << std::endl;
				}
				else
				{
					// if the detected class has no scale assigned in obj_class_scales
    				sample_obj->yolo_obj_scale = sample_obj -> scale;
				}
				// LL: Added by Leander
				
		
				sample_obj->rect_detect_2d = Vector4d(left_x_raw, top_y_raw, obj_width_raw, obj_height_raw);
				sample_obj->edge_distance_error = all_configs_error_one_objH(raw_cube_ind, 4); // record the original error
				sample_obj->edge_angle_error = all_configs_error_one_objH(raw_cube_ind, 5);
				sample_obj->normalized_error = normalized_score(box_id);
				double skew_ratio = sample_obj->scale.head(2).maxCoeff() / sample_obj->scale.head(2).minCoeff();
				sample_obj->skew_ratio = skew_ratio;
				sample_obj->down_expand_height = all_configs_error_one_objH(raw_cube_ind, 6);
				if (whether_sample_cam_roll_pitch)
				{
					sample_obj->camera_roll_delta = all_configs_error_one_objH(raw_cube_ind, 7) - cam_pose_raw.euler_angle(0);
					sample_obj->camera_pitch_delta = all_configs_error_one_objH(raw_cube_ind, 8) - cam_pose_raw.euler_angle(1);
				}
				else
				{
					sample_obj->camera_roll_delta = 0;
					sample_obj->camera_pitch_delta = 0;
				}

				raw_obj_proposals.push_back(sample_obj);
			}
		} // end of differnet object height sampling

		// %finally rank all proposals. [normalized_error   skew_error]
		int actual_cuboid_num_small = std::min(max_cuboid_num, (int)raw_obj_proposals.size());
		VectorXd all_combined_score(raw_obj_proposals.size());
		for (int box_id = 0; box_id < raw_obj_proposals.size(); box_id++)
		{
			cuboid *sample_obj = raw_obj_proposals[box_id];
			double skew_error = weight_skew_error * std::max(sample_obj->skew_ratio - nominal_skew_ratio, 0.0);
			if (sample_obj->skew_ratio > max_cut_skew)
				skew_error = 100;
			double new_combined_error = sample_obj->normalized_error + weight_skew_error * skew_error;
			all_combined_score(box_id) = new_combined_error;
		}

		std::vector<int> sort_idx_small(all_combined_score.rows());
		iota(sort_idx_small.begin(), sort_idx_small.end(), 0);
		sort_indexes(all_combined_score, sort_idx_small, actual_cuboid_num_small);
		for (int ii = 0; ii < actual_cuboid_num_small; ii++) // use sorted index
		{
			all_object_cuboids[object_id].push_back(raw_obj_proposals[sort_idx_small[ii]]);
		}

		ca::Profiler::tictoc("One 3D object total time");
	} // end of different objects

	if (whether_plot_final_images || whether_save_final_images)
	{
		cv::Mat frame_all_cubes_img = rgb_img.clone();
		for (int object_id = 0; object_id < all_object_cuboids.size(); object_id++)
			if (all_object_cuboids[object_id].size() > 0)
			{
				plot_image_with_cuboid(frame_all_cubes_img, all_object_cuboids[object_id][0]);
			}
		if (whether_save_final_images)
			cuboids_2d_img = frame_all_cubes_img;
		if (whether_plot_final_images)
		{
			cv::imshow("frame_all_cubes_img", frame_all_cubes_img);
			cv::waitKey(0);
		}
	}
}
#endif
// LL: Added by Leander