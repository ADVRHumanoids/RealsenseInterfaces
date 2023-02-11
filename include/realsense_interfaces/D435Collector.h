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

#include <ros/ros.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include "D435Interface.h"

namespace realsense_interfaces {

class D435Collector {

public:
    
    D435Collector(ros::NodeHandle* nh);
    bool init();
    bool run();
    
    rs2::context ctx;

    
private:
    ros::NodeHandle* _nh;
    
    std::map<std::string, std::unique_ptr<D435Interface>> _cams;
    std::shared_ptr<rs2::pointcloud> _pointcloud_filter;


};

} //namespace

#endif //D435_COLLECTOR_H

