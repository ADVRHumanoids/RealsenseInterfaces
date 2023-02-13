#include <realsense_interfaces/D435Collector.h>

using realsense_interfaces::D435Collector;

D435Collector::D435Collector(ros::NodeHandle* nh) : _nh(nh) {
    
    // Capture serial numbers before opening streaming
//     std::vector<std::string> serials;
//     for (auto&& dev : ctx.query_devices()) {
//         serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
//         std::cout << "a " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
//     } 
    
}

bool D435Collector::init() {
    
    //  _pointcloud_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("depth/color/points", 1);
    //    _pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("depth/color/points", 1);
    
    std::string point_cloud_topic;
    _nh->param<std::string>("point_cloud_topic", point_cloud_topic, "depth/color/points");
    _nh->param<std::string>("ref_frame", _ref_frame, "cams");
    _nh->param<bool>("publish_static_tf", _publish_static_tf, true);
    
    _pointcloud_publisher = _nh->advertise<realsense_interfaces::PointCloud>(point_cloud_topic, 1);

    //<rosparam param="x0">[0.0, 0.0]</rosparam>
    if (! _nh->getParam("camera_names", _camera_input_names)) {
        
        ROS_WARN("camera_names Params not found... setting example ones (2 cams)");
        _camera_input_names = {"cam_1", "cam_2"};
    } 
    
    if (! _nh->getParam("camera_serials", _camera_input_serials)) {
        
        ROS_WARN("camera_serials Params not found... setting example ones (2 cams)");
        _camera_input_serials = {"021222073482", "943222072865"};
    } 
    
    std::vector<double> ref_T_cams_numbers;
    std::vector<geometry_msgs::Transform> ref_T_cams;
    if (! _nh->getParam("ref_T_cams", ref_T_cams_numbers)) {
        
        ROS_WARN("ref_T_cams Params not found... setting example ones (2 cams)");
        ref_T_cams_numbers.resize(14);
        ref_T_cams_numbers.at(0) = 0;
        ref_T_cams_numbers.at(1) = 0.075;
        ref_T_cams_numbers.at(2) = 0;
        ref_T_cams_numbers.at(3) = 0;
        ref_T_cams_numbers.at(4) = 0;
        ref_T_cams_numbers.at(5) = 0;
        ref_T_cams_numbers.at(6) = 1;
        
        ref_T_cams_numbers.at(7) = 0;
        ref_T_cams_numbers.at(8) = -0.075;
        ref_T_cams_numbers.at(9) = 0;
        ref_T_cams_numbers.at(10) = 0;
        ref_T_cams_numbers.at(11) = 0;
        ref_T_cams_numbers.at(12) = 0;
        ref_T_cams_numbers.at(13) = 1;
    } 
    
    if (_camera_input_names.size() != _camera_input_serials.size()) {
        ROS_ERROR("camera_names and camera_serials have different sizes! (%ld and %ld)", 
                  _camera_input_names.size(), _camera_input_serials.size());
        return false;
    }
    
    if (ref_T_cams_numbers.size() != (_camera_input_names.size()*7)) {
        ROS_ERROR("ref_T_cams_numbers wrong number of elements, found %ld, expected %ld", 
                  ref_T_cams_numbers.size(), _camera_input_names.size()*7);
        return false;
    }
    
    for (size_t cam_i = 0; cam_i<_camera_input_names.size(); cam_i++) {
        unsigned int i = 0;
        geometry_msgs::Transform ref_T_cam;
        ref_T_cam.translation.x = ref_T_cams_numbers.at((i++)+(cam_i*7));
        ref_T_cam.translation.y = ref_T_cams_numbers.at((i++)+(cam_i*7));
        ref_T_cam.translation.z = ref_T_cams_numbers.at((i++)+(cam_i*7));
        ref_T_cam.rotation.x = ref_T_cams_numbers.at((i++)+(cam_i*7));
        ref_T_cam.rotation.y = ref_T_cams_numbers.at((i++)+(cam_i*7));
        ref_T_cam.rotation.z = ref_T_cams_numbers.at((i++)+(cam_i*7));
        ref_T_cam.rotation.w = ref_T_cams_numbers.at((i)+(cam_i*7));

        ref_T_cams.push_back(ref_T_cam);
    }
    
    for (size_t i=0; i<_camera_input_names.size(); i++) {
        
        auto it = _cams.insert(std::make_pair(
            _camera_input_names.at(i), 
            D435Interface(_camera_input_names.at(i), _camera_input_serials.at(i) ))
        );
        
        while (! it.first->second.init(ref_T_cams.at(i), _ref_frame)) {
            ROS_WARN_STREAM("[D435Collector::" << __func__ << "] Camera " << it.first->second.getCameraName()<< " failed to init, Retrying...");
        }
        
        if (_publish_static_tf) {
            _static_tf_broadcaster.sendTransform(*(it.first->second.getStaticTransforms()));
        }
    }
    
    _tfListener = std::make_unique<tf2_ros::TransformListener>(_tfBuffer);
    
    _image_transport = std::make_unique<image_transport::ImageTransport>(*_nh);
    
    for (size_t i = 0; i<_camera_input_names.size(); i++){
        _image_publishers.push_back(_image_transport->advertise(_camera_input_names.at(i) + "/color", 1));
    }
    
    return true;
}

bool D435Collector::start() { 
    
    for (auto &it : _cams){
        
        while (! it.second.start(_ctx)) {
            ROS_WARN_STREAM("[D435Collector::" << __func__ << "] Camera " << it.second.getCameraName()<< " failed to start, Retrying...");
        }

    }
    
    return true;
}


bool D435Collector::run() {
    
    //realsense sincro info
    // https://dev.intelrealsense.com/docs/multiple-depth-cameras-configuration#b-collecting-synchronous-frames
    
    bool first = true;
    bool ret = true;
    
    int i=0;
    for (auto &it : _cams) {
        
        geometry_msgs::TransformStamped ref_T_cam;
        if (it.second.isMovingCam()) {
            
            try {
                ref_T_cam = _tfBuffer.lookupTransform(_ref_frame, it.second.getCameraName(),
                                        ros::Time(0));
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                return false;
            }
            
            ret = ret && (it.second.update(&ref_T_cam.transform));
            
        } else {
            ret = ret && (it.second.update());
        }
        
        if (! ret) {
            return false;
        }
        
        if (first) {
            _final_cloud = *(it.second.getPointCloud()); 
            first = false;
        } else {
            
            _final_cloud += *(it.second.getPointCloud()); 
            
            //OR ??
            //pcl::concatenateFields (final_cloud, it.second.getPointCloudRGB(), final_cloud);
        } 

        _image_publishers.at(i).publish(it.second.getRosImage());
        
        i++;
    }

    _pointcloud_publisher.publish(_final_cloud);
    
    
    return ret;
    
}
