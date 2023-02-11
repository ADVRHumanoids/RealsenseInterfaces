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

#ifndef D435_INTERFACE_H
#define D435_INTERFACE_H

#include <string>
#include <memory>
#include <algorithm> 

#include <librealsense2/rs.hpp>
//#include <rs_sensor.h> //for rs2_extrinsics

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/Transform.h> //for pcl_ros::transrformPointCloud
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

namespace realsense_interfaces {
    
class D435Interface {
    
public:
    
    D435Interface(const std::string &camera_name, 
                  const std::string &serial_no);
    
    bool init(const rs2::context & ctx);
    
    bool update();
    
    virtual ~D435Interface() {};
    
    sensor_msgs::PointCloud2 getPointCloudMsg() const;
    pcl::PointCloud<pcl::PointXYZ> getPointCloud() const;
    pcl::PointCloud<pcl::PointXYZRGB> getPointCloudRGB() const;
    
private:
    const std::string _camera_name;
    const std::string _serial_no;
    
    //params originally get from launch file
    bool _ordered_pc;
    bool _align_depth;
    bool _allow_no_texture_points; //launch param, default to false
    const std::string _depth_optical_frame;
    const std::string _color_optical_frame;
    
    std::shared_ptr<rs2::pipeline> _pipeline;
    
    rs2::pointcloud _pointcloud;
    rs2::points _points;    
    
    //rs2_extrinsics _cam_T_ref;
    geometry_msgs::Transform _cam_T_ref;
    
    sensor_msgs::PointCloud2 _msg_pointcloud;
    pcl::PointCloud<pcl::PointXYZ> _pcl_pointcloud;
    pcl::PointCloud<pcl::PointXYZRGB> _pcl_pointcloudrgb;
    
    bool pointsToRosWithColors(const rs2::frameset& frameset);
    double frameSystemTimeSec(const rs2::frame& frame);
    bool setBaseTime(double frame_time, rs2_timestamp_domain time_domain);
    void reverse_memcpy(unsigned char* dst, const unsigned char* src, size_t n);
    
    bool transformPointCloud();
    
    std::shared_ptr<rs2::pointcloud> _pointcloud_filter;
    bool _is_initialized_time_base = false;
    ros::Time _ros_time_base;
    double _camera_time_base;
    
    
    /****************/
    std::shared_ptr<rs2::align> align_to_depth;
    rs2::colorizer colorizer;
    
    /************/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCL_Conversion(const rs2::points& points, const rs2::video_frame& color);
    std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
};

} //namespace

#endif //D435_INTERFACE_H
