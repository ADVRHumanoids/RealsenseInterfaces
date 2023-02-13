#include <realsense_interfaces/D435Interface2.h>

using realsense_interfaces::D435Interface2;

D435Interface2::D435Interface2(const std::string &camera_name, 
                  const std::string &serial_no) :
                  _camera_name(camera_name), _serial_no(serial_no),
                  _depth_optical_frame(_camera_name + "_depth_optical_frame"),
                  _color_optical_frame(_camera_name + "_color_optical_frame")

{
    _ordered_pc = true;
    _align_depth = true;
    _allow_no_texture_points = false; //launch param, default to false
                  
    align_to_depth = std::make_shared<rs2::align>(RS2_STREAM_DEPTH);
    
    _pointcloud_filter = std::make_shared<rs2::pointcloud>(RS2_STREAM_COLOR, 0);

}

bool D435Interface2::init(const geometry_msgs::Transform& cam_T_ref, const std::string& ref_frame) {
    
    _cam_T_ref = cam_T_ref;
    _ref_frame = ref_frame;
    
    return true;
}


bool D435Interface2::start(const rs2::context & ctx) {
    
    rs2::config cfg;
    cfg.enable_device(_serial_no);
    //cfg.enable_stream(RS2_STREAM_DEPTH, RS2_STREAM_COLOR);
    
    _pipeline = std::make_unique<rs2::pipeline>(ctx);
    auto pipe_profile = _pipeline->start(cfg);
    
    //pipeline = std::make_unique<rs2::pipeline>();
    //auto pipe_profile = pipeline->start();
    
 	try {
        auto fs = _pipeline->wait_for_frames(10000); //milliseconds
        //do nothign just check for first frames

    } catch ( std::runtime_error err) {

        ROS_ERROR("[%s] ERROR no message from camera for 10 sec", _camera_name.c_str());
        _pipeline->stop();
        return false;
    }

    ROS_INFO("[%s] ... first data arrived", _camera_name.c_str());

    return true;  
    
}

bool D435Interface2::update(const geometry_msgs::Transform* cam_T_ref) {
    
    rs2::frameset fs;
    if (! _pipeline->poll_for_frames(&fs)) {

        ROS_ERROR("[%s] ERROR : poll_for_frames returned false: no new frames set is available", _camera_name.c_str());
        return false;
    } else {
       // ROS_INFO("retrieved %ld frames", fs.size());
    }

    //fs = align_to_depth->process(fs);
    
    auto color = fs.get_color_frame();
    auto depth = fs.get_depth_frame();
    //auto depth_color = colorizer.colorize(depth);

    _pointcloud.map_to(color);

    _points = _pointcloud.calculate(depth);
    
    if (cam_T_ref != nullptr) {
        _cam_T_ref = *cam_T_ref;
    }

    ///////////// transform ??rs2_transform_point_to_point??
//     _points_transformed = _points;
//     
//     rs2::vertex* points_vertex = _points.get_vertices();
//     rs2::vertex* points_transformed_vertex = _points_transformed.get_vertices();
//     
//     for (size_t i=0; i<points_vertex.size(); i++) {
//         rs2_transform_point_to_point(points_transformed_vertex[i], _cam_T_ref , points_vertex[i]);
//     }
    
    ///////////////////////////////
    //pointsToRosWithColors(fs);
//////////////////////////
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = PCL_Conversion(_points, color);
    
    _pcl_pointcloudrgb = *cloud;
    
    _pcl_pointcloudrgb.header.frame_id = _ref_frame;
    
    auto tick = ros::Time::now();
    pcl_ros::transformPointCloud(_pcl_pointcloudrgb, _pcl_pointcloudrgb, _cam_T_ref);
    std::cout << "duration: " << (ros::Time::now() - tick).toSec() << std::endl;
    
    //transformPointCloud();
    
    
    
    /***********************/   
    
    return true;
}


bool D435Interface2::transformPointCloud() { 
        
    pcl::fromROSMsg(_msg_pointcloud,_pcl_pointcloud);
    
    pcl_ros::transformPointCloud(_pcl_pointcloud, _pcl_pointcloud, _cam_T_ref);
    
    pcl::toROSMsg(_pcl_pointcloud, _msg_pointcloud);


    return true;
}



bool D435Interface2::pointsToRosWithColors(const rs2::frameset& frameset) {
    
    double frame_time = frameset.get_timestamp();

    if (! _is_initialized_time_base )
    {
        _is_initialized_time_base = setBaseTime(frame_time, frameset.get_frame_timestamp_domain());
    }
    ros::Time t(frameSystemTimeSec(frameset));

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
            return false;
        }
        warn_count = 0;
    }
    
    int texture_width(0), texture_height(0);
    int num_colors(0);

    const rs2::vertex* vertex = _points.get_vertices();
    const rs2::texture_coordinate* color_point = _points.get_texture_coordinates();

    rs2_intrinsics depth_intrin = _points.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

    sensor_msgs::PointCloud2Modifier modifier(_msg_pointcloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(_points.size());
    if (_ordered_pc)
    {
        _msg_pointcloud.width = depth_intrin.width;
        _msg_pointcloud.height = depth_intrin.height;
        _msg_pointcloud.is_dense = false;
    }

    vertex = _points.get_vertices();
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
        color_point = _points.get_texture_coordinates();

        float color_pixel[2];
        for (size_t point_idx=0; point_idx < _points.size(); point_idx++, vertex++, color_point++)
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

        for (size_t point_idx=0; point_idx < _points.size(); point_idx++, vertex++)
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
    if (_align_depth) _msg_pointcloud.header.frame_id = _depth_optical_frame;
    else              _msg_pointcloud.header.frame_id = _color_optical_frame;
    if (!_ordered_pc)
    {
        _msg_pointcloud.width = valid_count;
        _msg_pointcloud.height = 1;
        _msg_pointcloud.is_dense = true;
        modifier.resize(valid_count);
    }
    
    return true;
}

double D435Interface2::frameSystemTimeSec(const rs2::frame& frame)
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

bool D435Interface2::setBaseTime(double frame_time, rs2_timestamp_domain time_domain)
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

void D435Interface2::reverse_memcpy(unsigned char* dst, const unsigned char* src, size_t n)
{
    size_t i;

    for (i=0; i < n; ++i)
        dst[n-1-i] = src[i];
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr D435Interface2::PCL_Conversion(const rs2::points& points, const rs2::video_frame& color){

    // Object Declaration (Point Cloud)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
    std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

    //================================
    // PCL Cloud Object Configuration
    //================================
    // Convert data captured from Realsense camera to Point Cloud
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    
    cloud->width  = static_cast<uint32_t>( sp.width()  );   
    cloud->height = static_cast<uint32_t>( sp.height() );
    cloud->is_dense = false;
    cloud->points.resize( points.size() );

    auto Texture_Coord = points.get_texture_coordinates();
    auto Vertex = points.get_vertices();

    // Iterating through all points and setting XYZ coordinates
    // and RGB values
    for (int i = 0; i < points.size(); i++)
    {   
        //===================================
        // Mapping Depth Coordinates
        // - Depth data stored as XYZ values
        //===================================
        cloud->points[i].x = Vertex[i].x;
        cloud->points[i].y = Vertex[i].y;
        cloud->points[i].z = Vertex[i].z;

        // Obtain color texture for specific point
        RGB_Color = RGB_Texture(color, Texture_Coord[i]);

        // Mapping Color (BGR due to Camera Model) -->TORI: not true, 2 1 0 order color are wrong
        cloud->points[i].r = std::get<0>(RGB_Color); // Reference tuple<2>
        cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
        cloud->points[i].b = std::get<2>(RGB_Color); // Reference tuple<0>

    }
    
   return cloud; // PCL RGB Point Cloud generated
}

std::tuple<int, int, int> D435Interface2::RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
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

sensor_msgs::PointCloud2 D435Interface2::getPointCloudMsg () const {
    return _msg_pointcloud;
}
pcl::PointCloud<pcl::PointXYZ> D435Interface2::getPointCloud () const {
    return _pcl_pointcloud;
}
pcl::PointCloud<pcl::PointXYZRGB> D435Interface2::getPointCloudRGB () const {
    return _pcl_pointcloudrgb;
}


std::string D435Interface2::getCameraName() const { return _camera_name; }

std::string D435Interface2::getSerial() const { return _serial_no; }

geometry_msgs::Transform D435Interface2::getCamTRef() const { return _cam_T_ref; }

std::string D435Interface2::getRefFrame() const { return _ref_frame; }
