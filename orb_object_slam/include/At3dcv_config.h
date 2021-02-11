#include <ros/ros.h>
#include <iostream>
// LL: Added macro to switch on Leander's code
#define at3dcv_andy
#define at3dcv_skip_rgbd_check
#define at3dcv_dyn_kpts_using_segmentation
// #define at3dcv_show_not_matched_dyn_kpts

// AC: not working yet as it is not testable since no people are considered to be dynamic
// #define at3dcv_dyn_obj_mapdrawer
#define at3dcv_tum
#define at3dcv_size
#define at3dcv_mask

// LL: at3dcv_no_depth is needed for both "..._all_proposals" and "..._best_proposal"
#define at3dcv_no_depth

// LL: Adopting the cost function for all cuboid propsels
// #define at3dcv_no_depth_all_proposals

// LL: Only evaluating the best ranked proposal, if good use it as landmark, if bad discard 
#define at3dcv_no_depth_best_proposal

std::string unix_stamp_as_identifier(ros::Time timestamp);