#include "At3dcv_config.h"
#include <cmath>
#include <ros/ros.h>
#include <iostream>

std::string unix_stamp_as_identifier(ros::Time timestamp)
{
    // LL: Retrive the seconds and nano seconds from the header stamp as stringstream
    std::stringstream time_sec;
    std::stringstream time_nsec;
    time_sec << timestamp.sec;
    time_nsec << timestamp.nsec;

    // LL: Some time stamps have less then 6 digits for the nano seconds which throws an error
    // LL: therefore we alongate it.
    int i=0;
    std::string time_nsec_long = time_nsec.str() + "000000";

    double double_last_val = 0;
    do {
        std::string string_last_val = time_nsec_long.substr(5-i, 1) +"."+time_nsec_long.substr(6-i, 3+i);
        double_last_val = atof(string_last_val.c_str());
        double_last_val = round(double_last_val);
        i++;

    }while (double_last_val == 10.0 && i < 5);

    std::string unix_file_name;

    if ( double_last_val != 10.0)
    {
        double_last_val = double_last_val * pow(10,i-1);
        unix_file_name = time_sec.str() + "." + time_nsec_long.substr(0,5-i+1) + std::to_string((int)double_last_val);
    }
    else
    {   
        int int_sec_last_value = std::stoi(time_sec.str().substr(9,1));
        int_sec_last_value = int_sec_last_value +1;
        unix_file_name = time_sec.str().substr(0,8)+ std::to_string(int_sec_last_value) + ".000000";
    }

    return unix_file_name;
}