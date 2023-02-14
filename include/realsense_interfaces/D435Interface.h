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
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cv_bridge/cv_bridge.h>

namespace realsense_interfaces {
    
typedef pcl::PointXYZ Point;
typedef pcl::PointXYZRGB PointColor;
typedef pcl::PointCloud<PointColor> PointCloud;   
    
class D435Interface {
    
public:
    
    D435Interface(const std::string &camera_name, 
                  const std::string &serial_no,
                  const bool &moving_cam=false
                 );
    
    bool init(const geometry_msgs::Transform & ref_T_cam, const std::string& ref_frame);
    bool start(const rs2::context & ctx);
    bool update(const geometry_msgs::Transform* ref_T_cam = nullptr);
    
    virtual ~D435Interface() {};
    
    std::string getCameraName() const;
    std::string getSerial() const;
    geometry_msgs::Transform getRefTCam() const;
    std::string getRefFrame() const;
    const std::vector<geometry_msgs::TransformStamped>* getStaticTransforms() const;
    bool isMovingCam() const;
    const sensor_msgs::ImageConstPtr getRosImage() const;
    const sensor_msgs::CameraInfoConstPtr getRosCameraInfoColor() const;
    const sensor_msgs::CameraInfoConstPtr getRosCameraInfoDepth() const;
        
    PointCloud::ConstPtr getPointCloud() const;
    
    void fromMsg(const geometry_msgs::Transform& in, tf2::Transform& out);
    void fromMsg(const geometry_msgs::Vector3& in, tf2::Vector3& out);
    void fromMsg(const geometry_msgs::Quaternion& in, tf2::Quaternion& out);

    
private:
    const std::string _camera_name;
    const std::string _serial_no;
    const std::string _depth_optical_frame;
    const std::string _color_optical_frame;
    const bool _moving_cam;
    const bool _allow_no_texture_points;

    std::shared_ptr<rs2::pipeline> _pipeline;
    
    rs2::pointcloud _pointcloud;
    rs2::points _points;    
    
    bool updateTransforms();
    geometry_msgs::Transform _ref_T_cam;
    std::string _ref_frame;
    tf2::Transform _cam_T_optical_tf;
    geometry_msgs::Transform _ref_T_optical;

    bool colorToRosImage(const rs2::video_frame& color);
    sensor_msgs::ImagePtr _ros_image;
    std::map<rs2_stream, sensor_msgs::CameraInfoPtr> _ros_camera_info;
    void fillCameraInfo(const rs2_intrinsics& intrinsic, const rs2_stream& type);

    PointCloud::Ptr  _pcl_pointcloud;
    bool pointsToPclColored(const rs2::video_frame& color);
    std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
    
    std::vector<geometry_msgs::TransformStamped> _static_transforms;
    void setStaticTransforms();
    
};

} //namespace

#endif //D435_INTERFACE_H
