
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <ros/ros.h>

#include <realsense_interfaces/D435Collector.h>

int main(int argc, char * argv[])
{
    
    // Init ROS node
    ros::init(argc, argv, "d435_main");
    ros::NodeHandle nh("~");

    double rate;
    nh.param<double>("rate", rate, 30);

    realsense_interfaces::D435Collector d435_collector(&nh);
    
    d435_collector.init();
    d435_collector.start();
    
    ros::Rate r(rate);
    while (ros::ok())
    {
        
        d435_collector.run();
        
        ros::spinOnce();
        r.sleep();

    }

    return 0;
}



