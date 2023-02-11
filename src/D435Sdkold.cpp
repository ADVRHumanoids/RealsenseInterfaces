
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <atomic> // for compare_exchange_strong, is it necessary to use this method???
#include <iostream>
#include <memory>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <algorithm>            // std::min, std::max

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

bool _ordered_pc = true;
bool _align_depth = true;
bool _allow_no_texture_points = false; //launch param, default to false

ros::Publisher _pointcloud_publisher;
sensor_msgs::PointCloud2 _msg_pointcloud;

std::shared_ptr<rs2::pointcloud> _pointcloud_filter;
bool _is_initialized_time_base(false);
ros::Time _ros_time_base;
double _camera_time_base;

std::string _camera_name = "D435_head_camera";
std::string depth_optical_frame = "D435_head_camera_depth_optical_frame"; 
std::string color_optical_frame = "D435_head_camera_color_optical_frame";


void reverse_memcpy(unsigned char* dst, const unsigned char* src, size_t n)
{
    size_t i;

    for (i=0; i < n; ++i)
        dst[n-1-i] = src[i];

}

bool setBaseTime(double frame_time, rs2_timestamp_domain time_domain)
{
    ROS_WARN_ONCE(time_domain == RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME ? "Frame metadata isn't available! (frame_timestamp_domain = RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME)" : "");
    if (time_domain == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
    {
        ROS_WARN("frame's time domain is HARDWARE_CLOCK. Timestamps may reset periodically.");
        _ros_time_base = ros::Time::now();
        _camera_time_base = frame_time;
        return true;
    }
    return false;
}

double frameSystemTimeSec(const rs2::frame& frame)
{
    if (frame.get_frame_timestamp_domain() == RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK)
    {
        double elapsed_camera_ms = (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base) / 1000.0;
        return (_ros_time_base.toSec() + elapsed_camera_ms);
    }
    else
    {
        return (frame.get_timestamp() / 1000.0);
    }
}

void publishPointCloud(rs2::points pc, const ros::Time& t, const rs2::frameset& frameset)
{
    if (0 == _pointcloud_publisher.getNumSubscribers())
        return;
    ROS_INFO_STREAM_ONCE("publishing " << (_ordered_pc ? "" : "un") << "ordered pointcloud.");

    
    rs2_stream texture_source_id = static_cast<rs2_stream>(_pointcloud_filter->get_option(rs2_option::RS2_OPTION_STREAM_FILTER));
    bool use_texture = texture_source_id != RS2_STREAM_ANY;
    static int warn_count(0);
    static const int DISPLAY_WARN_NUMBER(5);
    rs2::frameset::iterator texture_frame_itr = frameset.end();
    if (use_texture)
    {
        std::set<rs2_format> available_formats{ rs2_format::RS2_FORMAT_RGB8, rs2_format::RS2_FORMAT_Y8 };

        texture_frame_itr = std::find_if(frameset.begin(), frameset.end(), [&texture_source_id, &available_formats] (rs2::frame f)
                                {return (rs2_stream(f.get_profile().stream_type()) == texture_source_id) &&
                                            (available_formats.find(f.get_profile().format()) != available_formats.end()); });
        if (texture_frame_itr == frameset.end())
        {
            warn_count++;
            std::string texture_source_name = _pointcloud_filter->get_option_value_description(rs2_option::RS2_OPTION_STREAM_FILTER, static_cast<float>(texture_source_id));
            ROS_WARN_STREAM_COND(warn_count == DISPLAY_WARN_NUMBER, "No stream match for pointcloud chosen texture " << texture_source_name);
            return;
        }
        warn_count = 0;
    }
    
    int texture_width(0), texture_height(0);
    int num_colors(0);

    const rs2::vertex* vertex = pc.get_vertices();
    const rs2::texture_coordinate* color_point = pc.get_texture_coordinates();

    rs2_intrinsics depth_intrin = pc.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    sensor_msgs::PointCloud2Modifier modifier(_msg_pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(pc.size());
    if (_ordered_pc)
    {
        _msg_pointcloud.width = depth_intrin.width;
        _msg_pointcloud.height = depth_intrin.height;
        _msg_pointcloud.is_dense = false;
    }

    vertex = pc.get_vertices();
    size_t valid_count(0);
    
    if (use_texture)
    {
        rs2::video_frame texture_frame = (*texture_frame_itr).as<rs2::video_frame>();
        texture_width = texture_frame.get_width();
        texture_height = texture_frame.get_height();
        num_colors = texture_frame.get_bytes_per_pixel();
        uint8_t* color_data = (uint8_t*)texture_frame.get_data();
        std::string format_str;
        switch(texture_frame.get_profile().format())
        {
            case RS2_FORMAT_RGB8:
                format_str = "rgb";
                break;
            case RS2_FORMAT_Y8:
                format_str = "intensity";
                break;
            default:
                throw std::runtime_error("Unhandled texture format passed in pointcloud " + std::to_string(texture_frame.get_profile().format()));
        }
        _msg_pointcloud.point_step = addPointField(_msg_pointcloud, format_str.c_str(), 1, sensor_msgs::PointField::FLOAT32, _msg_pointcloud.point_step);
        _msg_pointcloud.row_step = _msg_pointcloud.width * _msg_pointcloud.point_step;
        _msg_pointcloud.data.resize(_msg_pointcloud.height * _msg_pointcloud.row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(_msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(_msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(_msg_pointcloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t>iter_color(_msg_pointcloud, format_str);
        color_point = pc.get_texture_coordinates();

        float color_pixel[2];
        for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++, color_point++)
        {
            float i(color_point->u);
            float j(color_point->v);
            bool valid_color_pixel(i >= 0.f && i <=1.f && j >= 0.f && j <=1.f);
            bool valid_pixel(vertex->z > 0 && (valid_color_pixel || _allow_no_texture_points));
            if (valid_pixel || _ordered_pc)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;

                if (valid_color_pixel)
                {
                    color_pixel[0] = i * texture_width;
                    color_pixel[1] = j * texture_height;
                    int pixx = static_cast<int>(color_pixel[0]);
                    int pixy = static_cast<int>(color_pixel[1]);
                    int offset = (pixy * texture_width + pixx) * num_colors;
                    reverse_memcpy(&(*iter_color), color_data+offset, num_colors);  // PointCloud2 order of rgb is bgr.
                }
                ++iter_x; ++iter_y; ++iter_z;
                ++iter_color;
                ++valid_count;
            }
        }
    }
    else
    {
    
        std::string format_str = "intensity";
        _msg_pointcloud.row_step = _msg_pointcloud.width * _msg_pointcloud.point_step;
        _msg_pointcloud.data.resize(_msg_pointcloud.height * _msg_pointcloud.row_step);

        sensor_msgs::PointCloud2Iterator<float>iter_x(_msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float>iter_y(_msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float>iter_z(_msg_pointcloud, "z");

        for (size_t point_idx=0; point_idx < pc.size(); point_idx++, vertex++)
        {
            bool valid_pixel(vertex->z > 0);
            if (valid_pixel || _ordered_pc)
            {
                *iter_x = vertex->x;
                *iter_y = vertex->y;
                *iter_z = vertex->z;

                ++iter_x; ++iter_y; ++iter_z;
                ++valid_count;
            }
        }
    }
    _msg_pointcloud.header.stamp = t;
    if (_align_depth) _msg_pointcloud.header.frame_id = depth_optical_frame;
    else              _msg_pointcloud.header.frame_id = color_optical_frame;
    if (!_ordered_pc)
    {
        _msg_pointcloud.width = valid_count;
        _msg_pointcloud.height = 1;
        _msg_pointcloud.is_dense = true;
        modifier.resize(valid_count);
    }
    _pointcloud_publisher.publish(_msg_pointcloud);
}

int main(int argc, char * argv[]) try
{
    
    // Init ROS node
    ros::init(argc, argv, "d435_tori");
    ros::NodeHandle nh("~");

    double rate;
    nh.param<double>("rate", rate, 30);
    
    _pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("depth/color/points", 1);

    rs2::context ctx;
    // Capture serial numbers before opening streaming
    std::vector<std::string> serials;
    for (auto&& dev : ctx.query_devices()) {
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        std::cout << "a " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
    } 
    
    
    rs2::config cfg;
    cfg.enable_device("021222073482");
    cfg.enable_stream(RS2_STREAM_DEPTH, RS2_STREAM_COLOR);
    
    //rs2::pipeline pipeline(ctx);
    //pipeline.start(cfg);
    rs2::pipeline pipeline;
    pipeline.start();
     
    rs2::pointcloud pc;
    rs2::points points;
    _pointcloud_filter = std::make_shared<rs2::pointcloud>(RS2_STREAM_COLOR, 0);

    ros::Rate r(rate);
    while (ros::ok())
    {
        
        rs2::frameset fs;
        if (! pipeline.poll_for_frames(&fs)) {

            ROS_WARN("[%s] ERROR : poll_for_frames returned false: no new frames set is available", _camera_name.c_str());
            continue;
        } else {
            ROS_WARN("retrieved %ld frames", fs.size());
        }

        auto color = fs.get_color_frame();

        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        auto depth = fs.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);
        
        /***********ROS STUFF*********/
        double frame_time = fs.get_timestamp();
        
//         bool placeholder_false(false);
//         if (_is_initialized_time_base.compare_exchange_strong(placeholder_false, true) )
//         {
//             _is_initialized_time_base = setBaseTime(frame_time, fs.get_frame_timestamp_domain());
//         }
        if (! _is_initialized_time_base )
        {
            _is_initialized_time_base = setBaseTime(frame_time, fs.get_frame_timestamp_domain());
        }
        
        ros::Time t(frameSystemTimeSec(fs));
        
        publishPointCloud(points, t ,fs);
        
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




