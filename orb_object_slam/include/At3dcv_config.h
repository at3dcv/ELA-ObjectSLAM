#include <ros/ros.h>
#include <iostream>
// LL: Added macro to switch on Leander's code
//#define at3dcv_leander
#define at3dcv_andy
// #define at3dcv_show_not_matched_dyn_kpts

// AC: not working yet as it is not testable since no people are considered to be dynamic
// #define at3dcv_dyn_obj_mapdrawer
#define at3dcv_tum
#define at3dcv_size

std::string unix_stamp_as_identifier(ros::Time timestamp);
