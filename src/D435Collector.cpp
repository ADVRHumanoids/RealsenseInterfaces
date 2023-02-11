#include <realsense_interfaces/D435Collector.h>

using realsense_interfaces::D435Collector;

D435Collector::D435Collector(ros::NodeHandle* nh) : _nh(nh) {
    
    //  _pointcloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("depth/color/points", 1);
//    _pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("depth/color/points", 1);
    _pointcloud_publisher = _nh->advertise<realsense_interfaces::PointCloud>("depth/color/points", 1);
    
    
    
            // Capture serial numbers before opening streaming
//     std::vector<std::string> serials;
//     for (auto&& dev : ctx.query_devices()) {
//         serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
//         std::cout << "a " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
//     } 

}

bool D435Collector::init() {
    
    _camera_input_names = {"cam_1", "cam_2"};
    _camera_input_serials = {"021222073482", "943222072865"};
    
    std::string _ref_frame = "cams";
    
    geometry_msgs::Transform ref_T_cam1, ref_T_cam2;
    ref_T_cam1.translation.x = 0;
    ref_T_cam1.translation.y = 0.075;
    ref_T_cam1.translation.z = 0;
    ref_T_cam1.rotation.x = 0;
    ref_T_cam1.rotation.y = 0;
    ref_T_cam1.rotation.z = 0;
    ref_T_cam1.rotation.w = 1;
    
    ref_T_cam2.translation.x = 0;
    ref_T_cam2.translation.y = -0.075;
    ref_T_cam2.translation.z = 0;
    ref_T_cam2.rotation = ref_T_cam1.rotation;
    
    std::vector<geometry_msgs::Transform> ref_T_cams;
    ref_T_cams.push_back(ref_T_cam1);
    ref_T_cams.push_back(ref_T_cam2);
    
    bool ret = true;

    for (size_t i=0; i<_camera_input_names.size(); i++) {
        
        auto it = _cams.insert(std::make_pair(
            _camera_input_names.at(i), 
            D435Interface(_camera_input_names.at(i), _camera_input_serials.at(i) ))
        );
        
        ret = ret && (it.first->second.init(ref_T_cams.at(i), _ref_frame));
        
        _static_tf_broadcaster.sendTransform(*(it.first->second.getStaticTransforms()));
    }
    
    return ret;
}

bool D435Collector::start() { 
    
    bool ret = true;
    for (auto &it : _cams){
        ret = ret && (it.second.start(_ctx));
    }
    
    return ret;
}


bool D435Collector::run() {
    
    //realsense sincro info
    // https://dev.intelrealsense.com/docs/multiple-depth-cameras-configuration#b-collecting-synchronous-frames
    
    bool first = true;
    bool ret = true;
    
    for (auto &it : _cams) {
        ret = ret && (it.second.update());
        
        if (first) {
            _final_cloud = *(it.second.getPointCloud()); 
            first = false;
        } else {
            
            _final_cloud += *(it.second.getPointCloud()); 
            
            //OR ??
            //pcl::concatenateFields (final_cloud, it.second.getPointCloudRGB(), final_cloud);
        } 
        
    }

    _pointcloud_publisher.publish(_final_cloud);
    
    
    return true;
    
}
