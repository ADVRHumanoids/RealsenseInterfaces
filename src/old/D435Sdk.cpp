
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <atomic> // for compare_exchange_strong, is it necessary to use this method???
#include <iostream>
#include <memory>
#include <algorithm>            // std::min, std::max

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <realsense_interfaces/D435Interface.h>

int main(int argc, char * argv[]) try
{
    
    // Init ROS node
    ros::init(argc, argv, "d435sdk");
    ros::NodeHandle nh("~");

    double rate;
    nh.param<double>("rate", rate, 30);
    
//     ros::Publisher pointcloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("depth/color/points", 1);
//     ros::Publisher pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("depth/color/points", 1);
    ros::Publisher pointcloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("depth/color/points", 1);
    
        // Capture serial numbers before opening streaming
//     std::vector<std::string> serials;
//     for (auto&& dev : ctx.query_devices()) {
//         serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
//         std::cout << "a " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
//     } 
    
    rs2::context ctx;
    
    realsense_interfaces::D435Interface d435("cam","021222073482");
    d435.init(ctx);

    ros::Rate r(rate);
    while (ros::ok())
    {
        
        d435.update();
        
        //pointcloud_publisher.publish(d435.getPointCloud());
//         pointcloud_publisher.publish(d435.getPointCloudMsg());
        pointcloud_publisher.publish(d435.getPointCloudRGB());

        ros::spinOnce();
        r.sleep();

    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}




