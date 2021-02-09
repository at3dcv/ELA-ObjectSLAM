#include <ros/ros.h>
#include <iostream>
// LL: Added macro to switch on Leander's code
//#define at3dcv_leander
#define at3dcv_andy
#define at3dcv_tum
#define at3dcv_size

// LL: at3dcv_no_depth is needed for both "..._all_proposals" and "..._best_proposal"
//#define at3dcv_no_depth

// LL: Adopting the cost function for all cuboid propsels
// #define at3dcv_no_depth_all_proposals

// LL: Only evaluating the best ranked proposal, if good use it as landmark, if bad discard 
//#define at3dcv_no_depth_best_proposal


