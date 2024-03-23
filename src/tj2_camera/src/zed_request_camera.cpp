#include <zed_request_camera.h>

ZedRequestCamera::ZedRequestCamera(ros::NodeHandle *nodehandle) : nh(*nodehandle),
                                                                  _image_transport(nh)
{
    ros::param::param<double>("~tick_rate", _tick_rate, 200.0);

    std::string camera_name;
    ros::param::param<std::string>("~camera_name", camera_name, "");

    int serial_number;
    ros::param::param<int>("~serial_number", serial_number, 0);

    _image_frame_id = camera_name + "_left_camera_optical_frame";

    _init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    _init_parameters.async_grab_camera_recovery = true;
    _init_parameters.sdk_verbose = false;
    _init_parameters.sdk_gpu_id = 0;
    _init_parameters.camera_image_flip = sl::FLIP_MODE::OFF;
    _init_parameters.coordinate_units = sl::UNIT::METER;

    std::string resolution_key;
    ros::param::param<std::string>("~resolution", resolution_key, "");

    sl::RESOLUTION resolution;
    int fps;
    int grab_fps = 15;
    if (resolution_key == "HD2K")
    {
        resolution = sl::RESOLUTION::HD2K;
        fps = 15;
    }
    else if (resolution_key == "HD1080")
    {
        resolution = sl::RESOLUTION::HD1080;
        fps = 30;
    }
    else if (resolution_key == "HD720")
    {
        resolution = sl::RESOLUTION::HD720;
        fps = 60;
    }
    else if (resolution_key == "VGA")
    {
        resolution = sl::RESOLUTION::VGA;
        fps = 100;
    }
    else
    {
        ROS_WARN("Not valid 'general.grab_resolution' value: '%s'. Using 'AUTO' setting.", resolution_key.c_str());
        resolution = sl::RESOLUTION::AUTO;
        fps = 15;
    }
    _init_parameters.camera_fps = fps;
    _init_parameters.grab_compute_capping_fps = static_cast<float>(fps);
    _init_parameters.camera_resolution = resolution;
    _init_parameters.input.setFromSerialNumber(serial_number);

    sl::DEPTH_MODE depth_mode;
    std::string depth_mode_str;
    ros::param::param<std::string>("~depth_mode", depth_mode_str, "PERFORMANCE");

    bool matched = false;
    for (int mode = static_cast<int>(sl::DEPTH_MODE::NONE); mode < static_cast<int>(sl::DEPTH_MODE::LAST); ++mode)
    {
        std::string test_str = sl::toString(static_cast<sl::DEPTH_MODE>(mode)).c_str();
        std::replace(test_str.begin(), test_str.end(), ' ',
                     '_'); // Replace spaces with underscores to match the YAML setting
        if (test_str == depth_mode_str)
        {
            matched = true;
            depth_mode = static_cast<sl::DEPTH_MODE>(mode);
            break;
        }
    }

    if (!matched)
    {
        ROS_WARN("The parameter 'depth_mode' contains a not valid string. Using default DEPTH_MODE.");
        depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    }

    _depth_enabled = depth_mode != sl::DEPTH_MODE::NONE;

    ros::param::param<int>("~depth_stabilization", _init_parameters.depth_stabilization, 1);

    float min_depth, max_depth;
    ros::param::param<float>("~min_depth", min_depth, 0.5);
    ros::param::param<float>("~max_depth", max_depth, 20.0);
    _init_parameters.depth_minimum_distance = min_depth;
    _init_parameters.depth_maximum_distance = max_depth;
    _init_parameters.open_timeout_sec = 1.0f;
    _init_parameters.enable_image_enhancement = true; // Always active

    _depth_pub = _image_transport.advertiseCamera("depth/image", 1);
    _image_pub = _image_transport.advertiseCamera("color/image", 1);
    _request_sub = nh.subscribe<tj2_interfaces::RequestFrames>("request", 1, &ZedRequestCamera::request_callback, this);
}

void ZedRequestCamera::request_callback(const tj2_interfaces::RequestFrames::ConstPtr &msg)
{
    _request_time = ros::Time::now();
    _request_duration = msg->request_duration;
}

void ZedRequestCamera::init_camera_info()
{
    int width = _zed.getCameraInformation().camera_configuration.resolution.width;
    int height = _zed.getCameraInformation().camera_configuration.resolution.height;
    _resolution = sl::Resolution(width, height);
    sl::CalibrationParameters zed_param = _zed.getCameraInformation(_resolution).camera_configuration.calibration_parameters;

    float baseline = zed_param.getCameraBaseline();
    _camera_info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    _camera_info->D.resize(5);
    _camera_info->D[0] = zed_param.left_cam.disto[0]; // k1
    _camera_info->D[1] = zed_param.left_cam.disto[1]; // k2
    _camera_info->D[2] = zed_param.left_cam.disto[4]; // k3
    _camera_info->D[3] = zed_param.left_cam.disto[2]; // p1
    _camera_info->D[4] = zed_param.left_cam.disto[3]; // p2

    _camera_info->K.fill(0.0);
    _camera_info->K[0] = static_cast<double>(zed_param.left_cam.fx);
    _camera_info->K[2] = static_cast<double>(zed_param.left_cam.cx);
    _camera_info->K[4] = static_cast<double>(zed_param.left_cam.fy);
    _camera_info->K[5] = static_cast<double>(zed_param.left_cam.cy);
    _camera_info->K[8] = 1.0;

    _camera_info->R.fill(0.0);
    for (size_t i = 0; i < 3; i++)
    {
        // identity
        _camera_info->R[i + i * 3] = 1;
    }

    _camera_info->P.fill(0.0);
    _camera_info->P[0] = static_cast<double>(zed_param.left_cam.fx);
    _camera_info->P[2] = static_cast<double>(zed_param.left_cam.cx);
    _camera_info->P[5] = static_cast<double>(zed_param.left_cam.fy);
    _camera_info->P[6] = static_cast<double>(zed_param.left_cam.cy);
    _camera_info->P[10] = 1.0;

    _camera_info->width = static_cast<uint32_t>(_resolution.width);
    _camera_info->height = static_cast<uint32_t>(_resolution.height);
    _camera_info->header.frame_id = _image_frame_id;
}

bool ZedRequestCamera::poll()
{
    sl::ERROR_CODE status = _zed.grab(_run_parameters);
    if (status != sl::ERROR_CODE::SUCCESS)
    {
        // Detect if a error occurred (for example: the zed have been disconnected) and re-initialize the ZED

        ROS_ERROR("Camera grab error: %s. Exiting.", sl::toString(status).c_str());
        return false;
    }
    ros::Time frame_timestamp = sl_time_to_ros(_zed.getTimestamp(sl::TIME_REFERENCE::IMAGE));

    sl::Mat mat_left;
    _zed.retrieveImage(mat_left, sl::VIEW::LEFT, sl::MEM::CPU, _resolution);
    sensor_msgs::ImagePtr image_msg = boost::make_shared<sensor_msgs::Image>();
    image_to_ros_msg(image_msg, mat_left, _image_frame_id, frame_timestamp);
    _image_pub.publish(image_msg, _camera_info);

    if (_depth_enabled)
    {
        sl::Mat mat_depth;
        _zed.retrieveMeasure(mat_depth, sl::MEASURE::DEPTH, sl::MEM::CPU, _resolution);
        sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
        image_to_ros_msg(depth_msg, mat_depth, _image_frame_id, frame_timestamp);
        _depth_pub.publish(depth_msg, _camera_info);
    }

    return true;
}

int ZedRequestCamera::init_camera()
{
    ROS_INFO("Opening ZED Camera...");

    sl::ERROR_CODE status = _zed.open(_init_parameters);
    if (status != sl::ERROR_CODE::SUCCESS)
    {
        ROS_ERROR("Camera open error: %s. Exiting.", sl::toString(status).c_str());
        return 1;
    }
    init_camera_info();
    return 0;
}

int ZedRequestCamera::run()
{
    int exit_code = init_camera();
    if (exit_code != 0)
    {
        return exit_code;
    }
    if (!_zed.isOpened())
    {
        ROS_ERROR("Camera not opened. Exiting.");
        return 1;
    }

    ros::Rate clock_rate(_tick_rate); // Hz
    while (ros::ok())
    {
        ros::spinOnce();
        clock_rate.sleep();
        if (ros::Time::now() - _request_time < _request_duration)
        {
            if (!poll())
            {
                return 1;
            }
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zed_request_camera");
    ros::NodeHandle nh;
    ZedRequestCamera node(&nh);
    return node.run();
}
