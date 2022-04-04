#include "tj2_limelight.h"

TJ2Limelight::TJ2Limelight(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _camera_info_manager(nh),
    _image_transport(nh)
{
    ros::param::param<string>("~nt_host", _nt_host, "127.0.0.1");
    ros::param::param<int>("~nt_port", _nt_port, 1735);
    ros::param::param<int>("~num_limelight_targets", _num_limelight_targets, 3);

    ros::param::param<string>("~video_url", _video_url, "");
    ros::param::param<string>("~camera_info_url", _camera_info_url, "");
    ros::param::param<string>("~frame_id", _frame_id, "camera_link");
    ros::param::param<string>("~base_frame_id", _base_frame, "turret_link");
    ros::param::param<double>("~max_frame_rate", _max_frame_rate, 30.0);
    ros::param::param<bool>("~publish_video", _publish_video, true);
    ros::param::param<double>("~field_vision_target_height_m", _field_vision_target_height, 0.0);
    ros::param::param<double>("~field_vision_target_distance_m", _field_vision_target_distance, 0.0);

    _nt = nt::GetDefaultInstance();
    nt::AddLogger(_nt,
                [](const nt::LogMessage& msg) {
                //   std::fputs(msg.message.c_str(), stderr);
                //   std::fputc('\n', stderr);
                    ROS_DEBUG("[NT]:\t %s", msg.message.c_str());
                },
                0, UINT_MAX
    );
    nt::StartClient(_nt, _nt_host.c_str(), _nt_port);
    ros::Duration(2.0).sleep(); // wait for table to populate

    _limelight_led_mode_entry = nt::GetEntry(_nt, "/limelight/ledMode");
    _limelight_cam_mode_entry = nt::GetEntry(_nt, "/limelight/camMode");
    _has_targets_entry = nt::GetEntry(_nt, "/limelight/tv");
    for (int index = 0; index < _num_limelight_targets; index++) {
        _tx_entries.push_back(nt::GetEntry(_nt, "/limelight/tx" + std::to_string(index)));
        _ty_entries.push_back(nt::GetEntry(_nt, "/limelight/ty" + std::to_string(index)));
        _thor_entries.push_back(nt::GetEntry(_nt, "/limelight/thor" + std::to_string(index)));
        _tvert_entries.push_back(nt::GetEntry(_nt, "/limelight/tvert" + std::to_string(index)));
    }
    _main_tx_entry = nt::GetEntry(_nt, "/limelight/tx");
    _main_ty_entry = nt::GetEntry(_nt, "/limelight/ty");

    _limelight_height_entry = nt::GetEntry(_nt, "/Preferences/Limelight Height");
    _limelight_angle_entry = nt::GetEntry(_nt, "/Preferences/Limelight Angle");
    _limelight_radius_entry = nt::GetEntry(_nt, "/Preferences/Limelight Radius");

    _limelight_raw_target_pub = nh.advertise<tj2_limelight::LimelightTargetArray>("raw_targets", 15);
    _limelight_target_pub = nh.advertise<geometry_msgs::PoseStamped>("target", 15);
    _limelight_led_mode_sub = nh.subscribe<std_msgs::Bool>("led_mode", 5, &TJ2Limelight::led_mode_callback, this);
    _limelight_cam_mode_sub = nh.subscribe<std_msgs::Bool>("cam_mode", 5, &TJ2Limelight::cam_mode_callback, this);

    _camera_info_manager.setCameraName("limelight");

    if (_camera_info_manager.validateURL(_camera_info_url))
    {
        if (_camera_info_manager.loadCameraInfo(_camera_info_url)) {
            ROS_INFO_STREAM("Loaded camera calibration from " << _camera_info_url);
        }
        else {
            ROS_WARN_STREAM("Could not load camera info, using an uncalibrated config.");
        }
    }
    else {
        ROS_WARN_STREAM("Given camera info url: " << _camera_info_url << " is not valid, using an uncalibrated config.");
    }

    if (_publish_video) {
        ROS_INFO_STREAM("Trying to connect to  " << _video_url);
        _video_capture.open(_video_url);
    }

    _out_msg.header.frame_id = _frame_id;
    _out_msg.encoding = sensor_msgs::image_encodings::BGR8;

    _camera_info = _camera_info_manager.getCameraInfo();
    _camera_info.header.frame_id = _frame_id;
    _camera_model.fromCameraInfo(_camera_info);

    _reopenSleep = ros::Duration(0.25);

    _camera_pub = _image_transport.advertiseCamera("camera/image", 10);

    if (_publish_video) {
        _watcher_thread = new boost::thread(&TJ2Limelight::watchVideoCapture, this);
    }

    ROS_INFO("tj2_limelight init complete");
}

void TJ2Limelight::watchVideoCapture()
{
    ros::Rate clock_rate(_max_frame_rate);  // Hz
    ros::Duration timeout(10.0);
    while (ros::ok())
    {
        clock_rate.sleep();
        if (_camera_pub.getNumSubscribers() <= 0) {
            continue;
        }
        if (ros::Time::now() - _last_publish_time > timeout) {
            reopenCapture();
        }
    }
}

double TJ2Limelight::getDouble(NT_Entry entry, double default_value)
{
    auto value = nt::GetEntryValue(entry);
    if (value && value->IsDouble()) {
        return value->GetDouble();
    }
    else {
        return default_value;
    }
}

void TJ2Limelight::publish_limelight_targets()
{
    if (getDouble(_has_targets_entry, 0.0) != 1.0) {
        return;
    }
    tj2_limelight::LimelightTargetArray array_msg;

    int width = _camera_info.width;
    int height = _camera_info.height;

    for (int index = 0; index < _num_limelight_targets; index++) {
        tj2_limelight::LimelightTarget target_msg;
        target_msg.thor = (int)(getDouble(_thor_entries.at(index), 0.0));
        target_msg.tvert = (int)(getDouble(_tvert_entries.at(index), 0.0));

        double tx = getDouble(_tx_entries.at(index), 0.0);
        double ty = getDouble(_ty_entries.at(index), 0.0);

        target_msg.tx = (int)((tx + 1.0) / 2.0 * width);
        target_msg.ty = (int)((-ty + 1.0) / 2.0 * height);
        array_msg.targets.push_back(target_msg);
    }
    _limelight_raw_target_pub.publish(array_msg);

    double main_tx = to_radians(getDouble(_main_tx_entry, 0.0));
    double main_ty = to_radians(getDouble(_main_ty_entry, 0.0));

    double limelight_height = to_meters(getDouble(_limelight_height_entry, 0.0));
    double limelight_angle = to_radians(getDouble(_limelight_angle_entry, 0.0));
    double limelight_radius = to_meters(getDouble(_limelight_radius_entry, 0.0));
    double target_dist = (_field_vision_target_height - limelight_height) /
                         (tan(limelight_angle + main_ty)
                         * cos(main_tx));

    double target_angle = atan(sin(main_tx) / (cos(main_tx) + limelight_radius / _field_vision_target_distance));

    tf2::Quaternion quat;
    quat.setRPY(0, 0, target_angle);
    geometry_msgs::Quaternion msg_quat = tf2::toMsg(quat);

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = _base_frame;
    pose.pose.position.x = target_dist * cos(target_angle);
    pose.pose.position.y = target_dist * sin(target_angle);
    pose.pose.orientation = msg_quat;

    _limelight_target_pub.publish(pose);
}
void TJ2Limelight::set_led_mode(bool mode)
{
    ROS_INFO("Setting limelight led mode to %d", mode);
    nt::SetEntryValue(_limelight_led_mode_entry, nt::Value::MakeDouble(mode ? 1.0 : 0.0));
}

void TJ2Limelight::led_mode_callback(const std_msgs::BoolConstPtr& msg)
{
    set_led_mode(msg->data);
}

void TJ2Limelight::cam_mode_callback(const std_msgs::BoolConstPtr& msg)
{
    ROS_INFO("Setting limelight cam mode to %d", msg->data);
    nt::SetEntryValue(_limelight_cam_mode_entry, nt::Value::MakeDouble(msg->data ? 1.0 : 0.0));
}

int TJ2Limelight::run()
{
    cv::Mat frame;
    ros::Rate loop(_max_frame_rate);
    while (ros::ok())
    {
        publish_limelight_targets();
        ros::spinOnce();
        loop.sleep();

        if (_publish_video) {
            if (_video_capture.isOpened())
            {
                ROS_INFO_ONCE("Connection established");
                if (!_video_capture.read(frame)) {
                    loop.sleep();
                    continue;
                }
                _last_publish_time = ros::Time::now();
                _out_msg.header.stamp = _last_publish_time;
                _out_msg.image = frame;
                _camera_info.header.stamp = _last_publish_time;

                _out_msg.toImageMsg(_ros_img);
                _camera_pub.publish(_ros_img, _camera_info, _last_publish_time);
            }
            else
            {
                reopenCapture();
            }
        }
    }
    return 0;
}

void TJ2Limelight::reopenCapture()
{
    ROS_WARN("Video stream is not available. Exiting.");
    _video_capture.release();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_limelight");
    ros::NodeHandle nh;
    TJ2Limelight node(&nh);
    return node.run();
}
