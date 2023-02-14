/*
 * Copyright 2021 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef D435_COLLECTOR_H
#define D435_COLLECTOR_H

#include <vector>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <realsense_interfaces/D435Interface.h>

namespace realsense_interfaces {

class D435Collector {

public:
    
    D435Collector(ros::NodeHandle* nh);
    bool init();
    bool start();
    bool run();
    
    rs2::context _ctx;

    
private:
    ros::NodeHandle* _nh;
    ros::Publisher _pointcloud_publisher;
    
    std::unique_ptr<image_transport::ImageTransport> _image_transport;
    std::vector<image_transport::Publisher> _image_publishers;
    std::vector<ros::Publisher> _camera_info_publishers;
    
    bool _publish_static_tf;
    tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;
    
    std::string _ref_frame;
    std::vector<std::string> _camera_input_names;
    std::vector<std::string> _camera_input_serials;
    
    std::map<std::string, realsense_interfaces::D435Interface> _cams;
    
    realsense_interfaces::PointCloud _final_cloud;
    
    //for moving _cams
    tf2_ros::Buffer _tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> _tfListener;


};

} //namespace

#endif //D435_COLLECTOR_H

