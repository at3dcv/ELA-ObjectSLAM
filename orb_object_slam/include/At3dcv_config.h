#include <ros/ros.h>
#include <iostream>
// LL: Added macro to switch on Leander's code
//#define at3dcv_leander
#define at3dcv_tum
#define at3dcv_tum_rgbd
//#define at3dcv_tum_mono
//#define at3dcv_andy

std::string unix_stamp_as_identifier(ros::Time timestamp);
