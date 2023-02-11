#include <realsense_interfaces/D435Collector.h>

using realsense_interfaces::D435Collector;

D435Collector::D435Collector(ros::NodeHandle* nh) : _nh(nh) {
    
    std::vector<std::string> camera_input_names{"cam_1", "cam_2"};
    std::vector<std::string> camera_input_serials{"021222073482", "943222072865"};
    
    _pointcloud_filter = std::make_shared<rs2::pointcloud>(RS2_STREAM_COLOR, 0);

}

bool D435Collector::init() {
    
    for (size_t i=0; i<camera_input_names.size(); i++) {
        
        _cams.insert(std::make_pair<std::string, D435Interface>(camera_input_names.at(i), D435Interface(it, camera_input_serials.at(i)));
    }
    
    return true;
}


bool D435Collector::run() {
    
    //realsense sincro 
    // https://dev.intelrealsense.com/docs/multiple-depth-cameras-configuration#b-collecting-synchronous-frames
    
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

    //ros::Time t(frameSystemTimeSec(fs));

    //publishPointCloud(points, t ,fs);
    
    return true;
    
}
