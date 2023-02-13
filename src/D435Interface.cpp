#include <realsense_interfaces/D435Interface.h>

using realsense_interfaces::D435Interface;

D435Interface::D435Interface(const std::string &camera_name, 
                  const std::string &serial_no, const bool &moving_cam) :
                  _camera_name(camera_name), _serial_no(serial_no),
                  _depth_optical_frame(_camera_name + "_depth_optical_frame"),
                  _color_optical_frame(_camera_name + "_color_optical_frame"),
                  _moving_cam(moving_cam),
                  _allow_no_texture_points(false)

{
}

/***
 * cam_T_ref is the trasf from camera to reference... then here we store the optical to ref since this is
 * necessary to transform the pointcloud wrt the ref frame
 */
bool D435Interface::init(const geometry_msgs::Transform& ref_T_cam, const std::string& ref_frame) {
    
    _ref_frame = ref_frame;
    _ref_T_cam = ref_T_cam;

    setStaticTransforms();
    
    bool flag = updateTransforms();
    
    return flag;
}

bool D435Interface::updateTransforms() {
    
    tf2::Transform ref_T_cam_tf, ref_T_optical_tf;
    D435Interface::fromMsg(_ref_T_cam, ref_T_cam_tf);
    
    ref_T_optical_tf = ref_T_cam_tf * _cam_T_optical_tf;
    
    _ref_T_optical = tf2::toMsg(ref_T_optical_tf);
    
    return true;
}


bool D435Interface::start(const rs2::context & ctx) {

    ROS_INFO("[%s] Starting...", _camera_name.c_str());

    rs2::config cfg;
    cfg.enable_device(_serial_no);

    //cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    //cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
    //cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

    _pipeline = std::make_unique<rs2::pipeline>(ctx);

    auto pipe_profile = _pipeline->start(cfg);
    
//     ROS_INFO("[%s] Resetting device...", _camera_name.c_str());
//     pipe_profile.get_device().hardware_reset();
//     ROS_INFO("[%s] ... Reset device DONE", _camera_name.c_str());
    
    rs2::device selected_device = pipe_profile.get_device();
    rs2::depth_sensor depth_sensor = selected_device.first<rs2::depth_sensor>();
    
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
    {
        std::cout << "Setting RS2_OPTION_EMITTER_ENABLED..." << std::endl;

       // depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
        //depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    {
        std::cout << "Setting RS2_OPTION_LASER_POWER..." << std::endl;
        // Query min and max values:
        //auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
        //depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
        //depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
    }
    
    ROS_INFO("[%s] ... waiting for first data...", _camera_name.c_str());
 	try {
        auto fs = _pipeline->wait_for_frames(5000); //milliseconds
        //do nothign just check for first frames

    } catch ( std::runtime_error err) {

        ROS_ERROR("[%s] ERROR no message from camera for 5 sec", _camera_name.c_str());
        _pipeline->stop();
        return false;
    }

    ROS_INFO("[%s] ... first data arrived", _camera_name.c_str());

    return true;  
    
}

bool D435Interface::update(const geometry_msgs::Transform* ref_T_cam) {
    
    rs2::frameset fs;
    if (! _pipeline->poll_for_frames(&fs)) {

        ROS_ERROR("[%s] ERROR : poll_for_frames returned false: no new frames set is available", _camera_name.c_str());
        return false;
    } else {
       // ROS_INFO("retrieved %ld frames", fs.size());
    }

    
    auto color = fs.get_color_frame();
    auto depth = fs.get_depth_frame();

    _pointcloud.map_to(color);

    _points = _pointcloud.calculate(depth);
    
    if (_moving_cam && ref_T_cam != nullptr) {
        _ref_T_cam = *ref_T_cam;
        updateTransforms();
    }
    
    pointsToPclColored(color);
    colorToRosImage(color);
    
    pcl_ros::transformPointCloud(*_pcl_pointcloud, *_pcl_pointcloud, _ref_T_optical);
     _pcl_pointcloud->header.frame_id = _ref_frame;
    
    return true;
}


bool D435Interface::pointsToPclColored(const rs2::video_frame& color){

    // TODO check if necessary to reset it by recreating...
    _pcl_pointcloud = boost::make_shared<PointCloud>();

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = _points.get_profile().as<rs2::video_stream_profile>();
    
    _pcl_pointcloud->width  = static_cast<uint32_t>( sp.width()  );   
    _pcl_pointcloud->height = static_cast<uint32_t>( sp.height() );
    _pcl_pointcloud->is_dense = false;
    _pcl_pointcloud->points.resize( _points.size() );

    const rs2::texture_coordinate* texture_coordinates = _points.get_texture_coordinates();
    const rs2::vertex* vertices = _points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (size_t i = 0; i < _points.size(); i++)
    {   
        if (!_allow_no_texture_points) {
            
            bool valid_color_pixel = 
                vertices[i].z > 0 &&
                texture_coordinates[i].u >= 0.f && texture_coordinates[i].u <=1.f &&
                texture_coordinates[i].v >= 0.f && texture_coordinates[i].v <=1.f;
                            
            if (!valid_color_pixel) {

                continue;
            }
        }
        
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        _pcl_pointcloud->points[i].x = vertices[i].x;
        _pcl_pointcloud->points[i].y = vertices[i].y;
        _pcl_pointcloud->points[i].z = vertices[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, texture_coordinates[i]);

        // Mapping Color (BGR due to Camera Model) -->TORI: not true, 2 1 0 order color are wrong
        _pcl_pointcloud->points[i].r = std::get<0>(RGB_Color); // Reference tuple<2>
        _pcl_pointcloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
        _pcl_pointcloud->points[i].b = std::get<2>(RGB_Color); // Reference tuple<0>

    }
    
    return true;
}


std::tuple<int, int, int> D435Interface::RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels
    
    // Normals to Texture Coordinates conversion
    int x_value = std::min(std::max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
    
    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}

bool D435Interface::colorToRosImage(const rs2::video_frame& color) {
    
    unsigned int width = color.get_width();
    unsigned int height = color.get_height();
    unsigned int bpp = color.get_bytes_per_pixel();
    
    cv::Mat cv_image;
    cv_image.create(height, width, CV_8UC3);
    cv_image.data = (uint8_t*)color.get_data();
    
    //TODO cam info
    
    
    _ros_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, cv_image).toImageMsg();
    _ros_image->width = width;
    _ros_image->height = height;
    _ros_image->is_bigendian = false;
    _ros_image->step = width * bpp;
    _ros_image->header.frame_id = _camera_name + "_color_optical_frame";
    _ros_image->header.stamp = ros::Time::now();//TODO;
    
    return true;
    
}


/***
 * Taken directly running official realsense ros launch and reading the tf. 
 * I am putting here only some of them because why all these tf are necessary?
 * Official ROS realsense code has a lot of if else to publish some trasf and it is a mess to understand what to pick and how,
 * but it is good because it takes values from the rs2 camera extrinsic param itself (from what I understood).
 */
void D435Interface::setStaticTransforms() {
    
    if (!_moving_cam) {
        geometry_msgs::TransformStamped ref_T_cam_stamp;
        ref_T_cam_stamp.header.frame_id = _ref_frame;
        ref_T_cam_stamp.child_frame_id = _camera_name;
        ref_T_cam_stamp.transform = _ref_T_cam;
        _static_transforms.push_back(ref_T_cam_stamp);
    }
    
    geometry_msgs::TransformStamped cam_T_depthOptical;
    cam_T_depthOptical.header.frame_id = _camera_name;
    cam_T_depthOptical.child_frame_id = _depth_optical_frame;
    cam_T_depthOptical.transform.translation.x = 0;
    cam_T_depthOptical.transform.translation.y = 0;
    cam_T_depthOptical.transform.translation.z = 0;
    cam_T_depthOptical.transform.rotation.x = -0.500;
    cam_T_depthOptical.transform.rotation.y = 0.500;
    cam_T_depthOptical.transform.rotation.z = -0.500;
    cam_T_depthOptical.transform.rotation.w = 0.500;
    _static_transforms.push_back(cam_T_depthOptical);
    
    //yes, sliglty no "nice" rotation for this... Error or not? who knows
    //https://github.com/IntelRealSense/realsense-ros/issues/2286
    geometry_msgs::TransformStamped cam_T_colorOptical;
    cam_T_colorOptical.header.frame_id = _camera_name;
    cam_T_colorOptical.child_frame_id = _color_optical_frame;
    cam_T_colorOptical.transform.translation.x = 0;
    cam_T_colorOptical.transform.translation.y = 0.015;
    cam_T_colorOptical.transform.translation.z = 0;
    cam_T_colorOptical.transform.rotation.x = -0.497;
    cam_T_colorOptical.transform.rotation.y = 0.501;
    cam_T_colorOptical.transform.rotation.z = -0.499;
    cam_T_colorOptical.transform.rotation.w = 0.503;
    _static_transforms.push_back(cam_T_colorOptical);
        
    //IDK if this is necessary...
    //notice that base_T_colorOptical the child is _camera_name + "_color__optical frame"
    geometry_msgs::TransformStamped cam_T_color;
    cam_T_color.header.frame_id = _camera_name;
    cam_T_color.child_frame_id = _camera_name + "_color_frame";
    cam_T_color.transform.translation.x = 0;
    cam_T_color.transform.translation.y = 0.015;
    cam_T_color.transform.translation.z = 0;
    cam_T_color.transform.rotation.x = 0.004;
    cam_T_color.transform.rotation.y = -0.002;
    cam_T_color.transform.rotation.z = 0;
    cam_T_color.transform.rotation.w = 1;
    _static_transforms.push_back(cam_T_color);
    
    
    //tf2::fromMsg(cam_T_colorOptical, _cam_T_optical_tf);
    D435Interface::fromMsg(cam_T_depthOptical.transform, _cam_T_optical_tf);
}

void D435Interface::fromMsg(const geometry_msgs::Transform& in, tf2::Transform& out)
{
  tf2::Vector3 v;
  tf2::fromMsg(in.translation, v);
  out.setOrigin(v);
  // w at the end in the constructor
  tf2::Quaternion q;
  tf2::fromMsg(in.rotation, q);
  out.setRotation(q);
}

void D435Interface::fromMsg(const geometry_msgs::Vector3& in, tf2::Vector3& out)
{
  out = tf2::Vector3(in.x, in.y, in.z);
}

void D435Interface::fromMsg(const geometry_msgs::Quaternion& in, tf2::Quaternion& out)
{
  // w at the end in the constructor
  out = tf2::Quaternion(in.x, in.y, in.z, in.w);
}

realsense_interfaces::PointCloud::ConstPtr D435Interface::getPointCloud () const {
    return _pcl_pointcloud;
}

std::string D435Interface::getCameraName() const { return _camera_name; }

std::string D435Interface::getSerial() const { return _serial_no; }

geometry_msgs::Transform D435Interface::getRefTCam() const { return _ref_T_cam; }

const std::vector<geometry_msgs::TransformStamped>* D435Interface::getStaticTransforms() const { return &_static_transforms; }

std::string D435Interface::getRefFrame() const { return _ref_frame; }

bool D435Interface::isMovingCam() const {return _moving_cam;}

const sensor_msgs::ImagePtr D435Interface::getRosImage() const {return _ros_image;}
