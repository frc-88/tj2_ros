#include "tj2_networktables.h"

TJ2NetworkTables::TJ2NetworkTables(ros::NodeHandle* nodehandle) :
    nh(*nodehandle), _tf_listener(_tf_buffer)
{
    ros::param::param<int>("~nt_port", _nt_port, 5800);
    ros::param::param<double>("~update_interval", _update_interval, 0.01);

    ros::param::param<bool>("~publish_odom_tf", _publish_odom_tf, true);
    ros::param::param<string>("~base_frame", _base_frame, "base_link");
    ros::param::param<string>("~odom_frame", _odom_frame, "odom");
    ros::param::param<string>("~map_frame", _map_frame, "map");
    ros::param::param<string>("~imu_frame", _imu_frame, "imu");

    ros::param::param<double>("~odom_timeout_warning", _odom_timeout_param, 0.1);

    ros::param::param<double>("~cmd_vel_timeout", _cmd_vel_timeout_param, 0.5);
    ros::param::param<double>("~min_linear_cmd", _min_linear_cmd, 0.05);
    ros::param::param<double>("~min_angular_z_cmd", _min_angular_z_cmd, 0.1);
    ros::param::param<double>("~zero_epsilon", _zero_epsilon, 0.001);

    double laser_angle_interval_degrees;
    ros::param::param<double>("~laser_angle_interval_degrees", laser_angle_interval_degrees, 45.0);
    _laser_angle_interval_rad = laser_angle_interval_degrees * M_PI / 180.0;

    double laser_angle_fan_degrees;
    ros::param::param<double>("~laser_angle_fan_degrees", laser_angle_fan_degrees, 10.0);
    _laser_angle_fan_rad = laser_angle_fan_degrees * M_PI / 180.0;
    
    ros::param::param<double>("~pose_estimate_x_std", _pose_estimate_x_std, 0.5);
    ros::param::param<double>("~pose_estimate_y_std", _pose_estimate_y_std, 0.5);
    ros::param::param<double>("~pose_estimate_theta_std_deg", _pose_estimate_theta_std_deg, 15.0);
    ros::param::param<string>("~pose_estimate_frame_id", _pose_estimate_frame_id, _map_frame);

    ros::param::param<std::string>("~classes_path", _classes_path, "coco.names");

    _class_names = load_label_names(_classes_path);

    _cmd_vel_timeout = ros::Duration(_cmd_vel_timeout_param);
    _odom_timeout = ros::Duration(_odom_timeout_param);

    string key;
    if (!ros::param::search("joint_names", key)) {
        ROS_ERROR("Failed to find joint_names parameter");
        std::exit(EXIT_FAILURE);
    }
    ROS_DEBUG("Found joint_names: %s", key.c_str());
    nh.getParam(key, _joint_names);

    _odom_covariance = get_double_list_param("odom_covariance", 36);
    _twist_covariance = get_double_list_param("twist_covariance", 36);
    _imu_orientation_covariance = get_double_list_param("imu_orientation_covariance", 9);
    _imu_angular_velocity_covariance = get_double_list_param("imu_angular_velocity_covariance", 9);
    _imu_linear_acceleration_covariance = get_double_list_param("imu_linear_acceleration_covariance", 9);

    _odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    _odom_msg.header.frame_id = _odom_frame;
    _odom_msg.child_frame_id = _base_frame;
    /* [
         0,  1,  2,  3,  4,  5,
         6,  7,  8,  9, 10, 11,
        12, 13, 14, 15, 16, 17,
        18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34, 35
    ] */
    _odom_msg.pose.covariance = as_array<36>(_odom_covariance);
    _odom_msg.twist.covariance = as_array<36>(_twist_covariance);

    _imu_pub = nh.advertise<tj2_interfaces::NavX>("imu", 50);
    _imu_msg.header.frame_id = _imu_frame;
    /* [
        0, 1, 2,
        3, 4, 5,
        6, 7, 8
    ] */
    // _imu_msg.orientation_covariance = as_array<9>(_imu_orientation_covariance);
    // _imu_msg.angular_velocity_covariance = as_array<9>(_imu_angular_velocity_covariance);
    // _imu_msg.linear_acceleration_covariance = as_array<9>(_imu_linear_acceleration_covariance);

    _raw_joint_pubs = new vector<ros::Publisher>();
    _raw_joint_subs = new vector<ros::Subscriber>();
    _raw_joint_msgs = new vector<std_msgs::Float64*>();
    
    _joint_commands.resize(_joint_names.size());
    for (size_t index = 0; index < _joint_names.size(); index++) {
        add_joint(_joint_names.at(index));
    }

    _match_time_pub = nh.advertise<std_msgs::Float64>("match_time", 10);
    _autonomous_pub = nh.advertise<std_msgs::Bool>("is_autonomous", 10);
    _team_color_pub = nh.advertise<std_msgs::String>("team_color", 10);

    _pose_estimate_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
    _pose_reset_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("reset_pose", 10);  // for distiguishing between ROS and RIO requested resets

    _nogo_zone_pub = nh.advertise<tj2_interfaces::NoGoZones>("nogo_zones", 10);

    _twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 50, &TJ2NetworkTables::twist_callback, this);
    _nt_passthrough_sub = nh.subscribe<tj2_interfaces::NTEntry>("nt_passthrough", 50, &TJ2NetworkTables::nt_passthrough_callback, this);
    _nt_passthrough_string_sub = nh.subscribe<tj2_interfaces::NTEntryString>("nt_passthrough_string", 50, &TJ2NetworkTables::nt_passthrough_string_callback, this);
    _waypoints_sub = nh.subscribe<tj2_interfaces::WaypointArray>("waypoints", 50, &TJ2NetworkTables::waypoints_callback, this);
    _field_relative_sub = nh.subscribe<std_msgs::Bool>("field_relative", 10, &TJ2NetworkTables::field_relative_callback, this);
    _laser_sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 10, &TJ2NetworkTables::scan_callback, this);
    _zones_sub = nh.subscribe<tj2_interfaces::ZoneInfoArray>("zones_info", 10, &TJ2NetworkTables::zones_info_callback, this);
    _tags_sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("tag_detections", 10, &TJ2NetworkTables::tags_callback, this);

    if (_class_names.empty()) {
        ROS_ERROR("Error loading class names! Not broadcasting detections");
    }
    else {
        _detections_sub = nh.subscribe<vision_msgs::Detection3DArray>("detections", 50, &TJ2NetworkTables::detections_callback, this);
    }

    _prev_twist_timestamp = ros::Time(0);
    _twist_cmd.resize(4);
    _twist_cmd[0] = 0.0;
    _twist_cmd[1] = 0.0;
    _twist_cmd[2] = 0.0;
    _twist_cmd[3] = 0.0;

    _global_pose.resize(4);
    _global_pose[0] = 0.0;
    _global_pose[1] = 0.0;
    _global_pose[2] = 0.0;
    _global_pose[3] = 0.0;

    int scan_size = (int)(2.0 * M_PI / _laser_angle_interval_rad);
    if (scan_size <= 0) {
        ROS_ERROR("NT Scan size is less than or equal to zero! Check your laser_angle_interval_degrees parmeter. %d", scan_size);
    }
    else {
        _laser_scan_xs.resize(scan_size);
        _laser_scan_ys.resize(scan_size);
        _laser_scan_ranges.resize(scan_size);
    }

    _ping_pub = nh.advertise<std_msgs::Float64>("ping", 50);
    _ping_timer = nh.createTimer(ros::Duration(0.5), &TJ2NetworkTables::ping_timer_callback, this);

    _nt = nt::GetDefaultInstance();
    nt::AddLogger(_nt,
                [](const nt::LogMessage& msg) {
                    // std::fputs(msg.message.c_str(), stderr);
                    // std::fputc('\n', stderr);
                    ROS_DEBUG("[NT]:\t %s", msg.message.c_str());
                },
                0, UINT_MAX
    );
    nt::StartServer(_nt, "tj2_networktables.ini", "", _nt_port);
    nt::SetUpdateRate(_nt, _update_interval);

    _base_key = "/ROS/";

    _ping_entry = nt::GetEntry(_nt, _base_key + "ping");
    _ping_return_entry = nt::GetEntry(_nt, _base_key + "ping_return");
    nt::AddEntryListener(_ping_return_entry, boost::bind(&TJ2NetworkTables::ping_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _odom_entry = nt::GetEntry(_nt, _base_key + "odom");
    nt::AddEntryListener(_odom_entry, boost::bind(&TJ2NetworkTables::odom_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _cmd_vel_entry = nt::GetEntry(_nt, _base_key + "cmd_vel");
    _field_relative_entry = nt::GetEntry(_nt, _base_key + "field_relative");

    _global_entry = nt::GetEntry(_nt, _base_key + "global");

    _pose_est_entry = nt::GetEntry(_nt, _base_key + "pose_est");
    nt::AddEntryListener(_pose_est_entry, boost::bind(&TJ2NetworkTables::pose_estimate_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _imu_entry = nt::GetEntry(_nt, _base_key + "imu");
    nt::AddEntryListener(_imu_entry, boost::bind(&TJ2NetworkTables::imu_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _match_time_entry = nt::GetEntry(_nt, _base_key + "match/time");
    _is_auto_entry = nt::GetEntry(_nt, _base_key + "match/is_auto");
    _team_color_entry = nt::GetEntry(_nt, _base_key + "match/team_color");
    nt::AddEntryListener(_match_time_entry, boost::bind(&TJ2NetworkTables::match_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _joint_states_entry = nt::GetEntry(_nt, _base_key + "joints/states");
    nt::AddEntryListener(_joint_states_entry, boost::bind(&TJ2NetworkTables::joint_state_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _joint_commands_entry = nt::GetEntry(_nt, _base_key + "joints/commands");
    nt::SetEntryValue(_joint_commands_entry, nt::Value::MakeDoubleArray(_joint_commands));

    string waypoints_base_key = _base_key + "waypoints/";
    _waypoint_names_entry = nt::GetEntry(_nt, waypoints_base_key + "name");
    _waypoint_xs_entry = nt::GetEntry(_nt, waypoints_base_key + "x");
    _waypoint_ys_entry = nt::GetEntry(_nt, waypoints_base_key + "y");
    _waypoint_ts_entry = nt::GetEntry(_nt, waypoints_base_key + "t");

    _laser_entry_xs = nt::GetEntry(_nt, _base_key + "laser/xs");
    _laser_entry_ys = nt::GetEntry(_nt, _base_key + "laser/ys");

    string zones_base_key = _base_key + "zones/";
    string zones_info_base_key = zones_base_key + "info/";
    _zones_valid_entry = nt::GetEntry(_nt, zones_info_base_key + "is_valid");
    _zone_names_entry = nt::GetEntry(_nt, zones_info_base_key + "names");
    _zone_nearest_x_entry = nt::GetEntry(_nt, zones_info_base_key + "nearest_x");
    _zone_nearest_y_entry = nt::GetEntry(_nt, zones_info_base_key + "nearest_y");
    _zone_distance_entry = nt::GetEntry(_nt, zones_info_base_key + "distance");
    _zone_is_inside_entry = nt::GetEntry(_nt, zones_info_base_key + "is_inside");
    _zone_is_nogo_entry = nt::GetEntry(_nt, zones_info_base_key + "is_nogo");
    _nogo_zones_names_entry = nt::GetEntry(_nt, zones_base_key + "set_nogo");
    _nogo_zones_update_entry = nt::GetEntry(_nt, zones_base_key + "update");
    nt::AddEntryListener(_nogo_zones_update_entry, boost::bind(&TJ2NetworkTables::nogo_zones_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    ROS_INFO("tj2_networktables init complete");
}

// ---
// NT publishers
// ---
void TJ2NetworkTables::publish_cmd_vel()
{
    ros::Duration dt = ros::Time::now() - _prev_twist_timestamp;
    if (dt > _cmd_vel_timeout) {
        ROS_DEBUG_THROTTLE(5.0, "cmd_vel timed out skipping write.");
        return;
    }

    nt::SetEntryValue(_cmd_vel_entry, nt::Value::MakeDoubleArray(_twist_cmd));
}

void TJ2NetworkTables::publish_robot_global_pose()
{
    geometry_msgs::TransformStamped transform;
    try {
        transform = _tf_buffer.lookupTransform(_map_frame, _base_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "Failed to publish robot's global pose: %s", ex.what());
        return;
    }

    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;

    tf2::Quaternion quat;
    tf2::convert(transform.transform.rotation, quat);
    tf2::Matrix3x3 m1(quat);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    
    _global_pose[0] = get_time();
    _global_pose[1] = x;
    _global_pose[2] = y;
    _global_pose[3] = yaw;

    nt::SetEntryValue(_global_entry, nt::Value::MakeDoubleArray(_global_pose));
}

// ---
// Subscription callbacks
// ---
void TJ2NetworkTables::twist_callback(const geometry_msgs::TwistConstPtr& msg)
{
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double vt = msg->angular.z;
    
    // If magnitude of translation is in the "no-go" zone (_zero_epsilon..._min_linear_cmd),
    // set vx, vy to _min_linear_cmd with heading applied
    double trans_vel = sqrt(vx * vx + vy * vy);
    if (_zero_epsilon < abs(trans_vel) && abs(trans_vel) < _min_linear_cmd)
    {
        double trans_angle = atan2(vy, vx);
        vx = _min_linear_cmd * cos(trans_angle);
        vy = _min_linear_cmd * sin(trans_angle);
    }
    // If magnitude of translation is in the "zero" zone (<_zero_epsilon),
    // Set translation velocity to zero
    //      If angular velocity is in the "no-go" zone,
    //      set vt to _min_angular_z_cmd with direction applied
    //      If angular velocity is in the "zero" zone,
    //      set vt to zero
    else if (abs(trans_vel) < _zero_epsilon) {
        vx = 0.0;
        vy = 0.0;
        if (_zero_epsilon < abs(vt) && abs(vt) < _min_angular_z_cmd) {
            vt = sign_of(vt) * _min_angular_z_cmd;
        }
        else if (abs(vt) < _zero_epsilon) {
            vt = 0.0;
        }
    }

    _prev_twist_timestamp = ros::Time::now();
    _twist_cmd[0] = get_time();
    _twist_cmd[1] = vx;
    _twist_cmd[2] = vy;
    _twist_cmd[3] = vt;
}

void TJ2NetworkTables::nt_passthrough_callback(const tj2_interfaces::NTEntryConstPtr& msg)
{
    NT_Entry entry = nt::GetEntry(_nt, _base_key + msg->path);
    nt::SetEntryValue(entry, nt::Value::MakeDouble(msg->value));
}

void TJ2NetworkTables::nt_passthrough_string_callback(const tj2_interfaces::NTEntryStringConstPtr& msg)
{
    NT_Entry entry = nt::GetEntry(_nt, _base_key + msg->path);
    nt::SetEntryValue(entry, nt::Value::MakeString(msg->value));
}

void TJ2NetworkTables::waypoints_callback(const tj2_interfaces::WaypointArrayConstPtr& msg)
{
    _waypoint_names.resize(msg->waypoints.size());
    _waypoint_xs.resize(msg->waypoints.size());
    _waypoint_ys.resize(msg->waypoints.size());
    _waypoint_ts.resize(msg->waypoints.size());
    for (size_t index = 0; index < msg->waypoints.size(); index++)
    {
        string name = msg->waypoints.at(index).name;
        geometry_msgs::Pose pose = msg->waypoints.at(index).pose;
        double x = pose.position.x;
        double y = pose.position.y;

        tf2::Quaternion quat(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        tf2::Matrix3x3 m1(quat);
        double roll, pitch, yaw;
        m1.getRPY(roll, pitch, yaw);

        _waypoint_names[index] = name;
        _waypoint_xs[index] = x;
        _waypoint_ys[index] = y;
        _waypoint_ts[index] = yaw;
    }
    nt::SetEntryValue(_waypoint_names_entry, nt::Value::MakeStringArray(_waypoint_names));
    nt::SetEntryValue(_waypoint_xs_entry, nt::Value::MakeDoubleArray(_waypoint_xs));
    nt::SetEntryValue(_waypoint_ys_entry, nt::Value::MakeDoubleArray(_waypoint_ys));
    nt::SetEntryValue(_waypoint_ts_entry, nt::Value::MakeDoubleArray(_waypoint_ts));
}

void TJ2NetworkTables::field_relative_callback(const std_msgs::BoolConstPtr& msg)
{
    nt::SetEntryValue(_field_relative_entry, nt::Value::MakeBoolean(msg->data));
}

void TJ2NetworkTables::scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
    if (_laser_scan_xs.size() == 0) {
        ROS_WARN_THROTTLE(1.0, "NT Laser x size is zero! Check your laser_angle_interval_degrees parmeter. %lu", _laser_scan_xs.size());
        return;
    }

    geometry_msgs::TransformStamped transform;
    try {
        transform = _tf_buffer.lookupTransform(_base_frame, msg->header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "Failed to transform laser to base frame: %s", ex.what());
        return;
    }

    for (size_t index = 0; index < _laser_scan_ranges.size(); index++) {
        _laser_scan_ranges.at(index) = msg->range_max;
    }

    double wrap_angle = msg->angle_max - msg->angle_min + msg->angle_increment;

    for (size_t index = 0; index < msg->ranges.size(); index++) {
        double laser_angle = msg->angle_increment * index + msg->angle_min;
        for (size_t copy_index = 0; copy_index < _laser_scan_ranges.size(); copy_index++) {
            double lower_angle = _laser_angle_interval_rad * copy_index + msg->angle_min - _laser_angle_fan_rad;
            double upper_angle = _laser_angle_interval_rad * copy_index + msg->angle_min + _laser_angle_fan_rad;
            
            bool is_within_angle = false;
            if (lower_angle < msg->angle_min || upper_angle > msg->angle_max) {
                if (lower_angle < msg->angle_min) {
                    lower_angle += wrap_angle;
                }
                if (upper_angle > msg->angle_max) {
                    upper_angle -= wrap_angle;
                }
                is_within_angle = !(upper_angle < laser_angle && laser_angle < lower_angle);
            }
            else {
                is_within_angle = lower_angle <= laser_angle && laser_angle <= upper_angle;
            }

            if (is_within_angle) {
                double distance = msg->ranges.at(index);
                if (msg->range_min <= distance && distance <= _laser_scan_ranges.at(copy_index)) {
                    _laser_scan_ranges.at(copy_index) = distance;
                }
            }
        }
    }

    geometry_msgs::PoseStamped laser_pose, base_pose;
    laser_pose.header.frame_id = msg->header.frame_id;
    laser_pose.pose.orientation.w = 1.0;

    for (size_t index = 0; index < _laser_scan_ranges.size(); index++) {
        double angle = _laser_angle_interval_rad * index + msg->angle_min;
        double distance = _laser_scan_ranges.at(index);

        laser_pose.pose.position.x = distance * cos(angle);
        laser_pose.pose.position.y = distance * sin(angle);

        tf2::doTransform(laser_pose, base_pose, transform);
    
        _laser_scan_xs.at(index) = base_pose.pose.position.x;
        _laser_scan_ys.at(index) = base_pose.pose.position.y;
    }

    nt::SetEntryValue(_laser_entry_xs, nt::Value::MakeDoubleArray(_laser_scan_xs));
    nt::SetEntryValue(_laser_entry_ys, nt::Value::MakeDoubleArray(_laser_scan_ys));
}

void TJ2NetworkTables::zones_info_callback(const tj2_interfaces::ZoneInfoArrayConstPtr& msg)
{
    nt::SetEntryValue(_zones_valid_entry, nt::Value::MakeBoolean(msg->is_valid));
    if (!msg->is_valid) {
        return;
    }
    
    vector<string> names;
    vector<double> nearest_xs;
    vector<double> nearest_ys;
    vector<double> distances;
    vector<double> is_insides;
    vector<double> is_nogos;

    for (size_t index = 0; index < msg->zones.size(); index++) {
        tj2_interfaces::ZoneInfo zone = msg->zones.at(index);
        names.push_back(zone.zone.name);
        nearest_xs.push_back(zone.nearest_point.x);
        nearest_ys.push_back(zone.nearest_point.y);
        distances.push_back(zone.distance);
        is_insides.push_back(zone.is_inside ? 1.0 : 0.0);
        is_nogos.push_back(zone.is_nogo ? 1.0 : 0.0);
    }
    nt::SetEntryValue(_zone_names_entry, nt::Value::MakeStringArray(names));
    nt::SetEntryValue(_zone_nearest_x_entry, nt::Value::MakeDoubleArray(nearest_xs));
    nt::SetEntryValue(_zone_nearest_y_entry, nt::Value::MakeDoubleArray(nearest_ys));
    nt::SetEntryValue(_zone_distance_entry, nt::Value::MakeDoubleArray(distances));
    nt::SetEntryValue(_zone_is_inside_entry, nt::Value::MakeDoubleArray(is_insides));
    nt::SetEntryValue(_zone_is_nogo_entry, nt::Value::MakeDoubleArray(is_nogos));
}

void TJ2NetworkTables::detections_callback(const vision_msgs::Detection3DArrayConstPtr& msg)
{
    if (_class_names.empty()) {
        ROS_ERROR("No class names loaded! Not broadcasting detections");
        return;
    }
    for (size_t index = 0; index < _class_names.size(); index++) {
        _detection_counter[_class_names.at(index)] = 0;
    }
    for (size_t index = 0; index < msg->detections.size(); index++) {
        vision_msgs::ObjectHypothesisWithPose hyp = msg->detections.at(index).results[0];
        string name = get_label(hyp.id);
        _detection_counter[name]++;
        // int obj_index = get_index(hyp.id);
        geometry_msgs::PoseStamped pose;
        pose.pose = hyp.pose.pose;
        pose.header = msg->header;
        publish_detection(name, index, pose);
    }
    for (size_t index = 0; index < _class_names.size(); index++) {
        publish_detection_count(_class_names.at(index), _detection_counter[_class_names.at(index)]);
    }
}

void TJ2NetworkTables::joint_command_callback(const std_msgs::Float64ConstPtr& msg, int joint_index)
{
    if ((size_t)joint_index < _joint_commands.size()) {
        _joint_commands[joint_index] = msg->data;
        nt::SetEntryValue(_joint_commands_entry, nt::Value::MakeDoubleArray(_joint_commands));
    }
    else {
        ROS_WARN_THROTTLE(1.0, "Received invalid command for joint index: %d", joint_index);
    }
}

void TJ2NetworkTables::tags_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
    std::set<string> found_ids;
    for (size_t index = 0; index < msg->detections.size(); index++) {
        apriltag_ros::AprilTagDetection detection = msg->detections.at(index);
        string name = "";
        for (size_t id_index = 0; id_index < detection.id.size(); id_index++) {
            name += to_string(detection.id.at(id_index));
            if (id_index != detection.id.size() - 1) {
                name += "-";
            }
        }
        found_ids.insert(name);
        if (!_detection_counter.count(name)) {
            _detection_counter[name] = 0;
        }
        geometry_msgs::PoseStamped pose;
        pose.pose = detection.pose.pose.pose;
        pose.header = detection.pose.header;
        publish_detection(name, _detection_counter[name], pose);
        _detection_counter[name]++;
    }
    for (string name : found_ids) {
        publish_detection_count(name, _detection_counter[name]);
        _detection_counter[name] = 0;
    }
}

// ---
// NT callbacks
// ---

void TJ2NetworkTables::odom_callback(const nt::EntryNotification& event)
{
    _prev_odom_timestamp = ros::Time::now();
}

void TJ2NetworkTables::ping_callback(const nt::EntryNotification& event)
{
    double sent_time = get_double(event.entry, NAN);
    if (!std::isfinite(sent_time)) {
        ROS_WARN_THROTTLE(1.0, "Ping time is nan or inf");
        return;
    }
    double ping = get_time() - sent_time;
    std_msgs::Float64 msg;
    msg.data = ping;
    _ping_pub.publish(msg);
}

void TJ2NetworkTables::imu_callback(const nt::EntryNotification& event)
{
    _prev_imu_timestamp = ros::Time::now();
}

void TJ2NetworkTables::publish_joint(size_t joint_index, double joint_position)
{
    if (joint_index >= _raw_joint_msgs->size()) {
        ROS_WARN_THROTTLE(1.0, "Invalid joint index received: %ld. Valid range is 0..%ld. (Joint value was %f)", joint_index, _raw_joint_msgs->size() - 1, joint_position);
        return;
    }
    if (!std::isfinite(joint_position)) {
        ROS_WARN_THROTTLE(1.0, "Joint position for index %ld is nan or inf", joint_index);
        joint_position = 0.0;
    }
    std_msgs::Float64* msg = _raw_joint_msgs->at(joint_index);
    msg->data = joint_position;

    _raw_joint_pubs->at(joint_index).publish(*msg);
}

void TJ2NetworkTables::match_callback(const nt::EntryNotification& event)
{
    std_msgs::Float64 timer_msg;
    timer_msg.data = get_double(_match_time_entry, -1.0);
    _match_time_pub.publish(timer_msg);

    std_msgs::Bool is_auto_msg;
    is_auto_msg.data = get_boolean(_is_auto_entry, false);
    _autonomous_pub.publish(is_auto_msg);

    std_msgs::String team_color_msg;
    team_color_msg.data = get_string(_team_color_entry, "");
    _team_color_pub.publish(team_color_msg);
}

void TJ2NetworkTables::pose_estimate_callback(const nt::EntryNotification& event)
{
    geometry_msgs::PoseWithCovarianceStamped pose_est;

    /*
    [
        x_std * x_std, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, y_std * y_std, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, theta_std_rad * theta_std_rad
    ]
    [
         0,  1,  2,  3,  4,  5,
         6,  7,  8,  9, 10, 11,
        12, 13, 14, 15, 16, 17,
        18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34, 35
    ]
    */

    vector<double> pose_est_value = get_double_array(_pose_est_entry);
    if (pose_est_value.size() != 4) {
        ROS_WARN_THROTTLE(1.0, "Received invalid size for pose estimate: %lu. Ignoring.", pose_est_value.size());
        return;
    }

    // index 0 is timestamp
    double x = pose_est_value[1];
    double y = pose_est_value[2];
    double theta = pose_est_value[3];

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta);

    geometry_msgs::Quaternion msg_quat = tf2::toMsg(quat);

    pose_est.pose.pose.position.x = x;
    pose_est.pose.pose.position.y = y;
    pose_est.pose.pose.orientation = msg_quat;
    pose_est.header.frame_id = _pose_estimate_frame_id;

    double theta_std_rad = _pose_estimate_theta_std_deg * M_PI / 180.0;

    pose_est.pose.covariance[0] = _pose_estimate_x_std * _pose_estimate_x_std;
    pose_est.pose.covariance[7] = _pose_estimate_y_std * _pose_estimate_y_std;
    pose_est.pose.covariance[35] = theta_std_rad * theta_std_rad;

    _pose_estimate_pub.publish(pose_est);
    _pose_reset_pub.publish(pose_est);
}


void TJ2NetworkTables::joint_state_callback(const nt::EntryNotification& event)
{
    vector<double> states = get_double_array(_joint_states_entry);
    for (size_t index = 0; index < states.size(); index++) {
        publish_joint(index, states[index]);
    }
}

void TJ2NetworkTables::nogo_zones_callback(const nt::EntryNotification& event)
{
    tj2_interfaces::NoGoZones msg;
    vector<string> nogos = get_string_array(_nogo_zones_names_entry);
    msg.nogo.resize(nogos.size());
    for (size_t index = 0; index < nogos.size(); index++) {
        msg.nogo.at(index) = std_msgs::String();
        msg.nogo.at(index).data = nogos[index];
    }
    _nogo_zone_pub.publish(msg);
}

// ---
// Timer callbacks
// ---
void TJ2NetworkTables::ping_timer_callback(const ros::TimerEvent& event)
{
    nt::SetEntryValue(_ping_entry, nt::Value::MakeDouble(get_time()));
}


// ---
// Other helpers
// ---

void TJ2NetworkTables::add_joint(string name)
{
    size_t joint_index = _raw_joint_pubs->size();
    ROS_INFO("Publishing to joint topic: %s. Index: %ld", name.c_str(), joint_index);
    _raw_joint_pubs->push_back(nh.advertise<std_msgs::Float64>("joint/" + name, 50));
    _raw_joint_msgs->push_back(new std_msgs::Float64);

    _raw_joint_subs->push_back(
        nh.subscribe<std_msgs::Float64>(
            "joint/command/" + name, 50,
            boost::bind(&TJ2NetworkTables::joint_command_callback, this, _1, joint_index)
        )
    );
}

double TJ2NetworkTables::get_time() {
    return ros::Time::now().toSec();
}

vector<double> TJ2NetworkTables::get_double_list_param(string name, size_t length)
{
    string key;
    vector<double> double_list;
    if (!ros::param::search(name, key)) {
        ROS_ERROR("Failed to find odom_covariance parameter");
        std::exit(EXIT_FAILURE);
    }
    ROS_DEBUG("Found %s: %s", name.c_str(), key.c_str());
    nh.getParam(key, double_list);
    if (double_list.size() != length) {
        ROS_ERROR("%s is not length %ld", name.c_str(), length);
        std::exit(EXIT_FAILURE);
    }
    return double_list;
}

string TJ2NetworkTables::get_label(int obj_id)
{
    size_t index = (size_t)(obj_id & 0xffff);
    if (index < _class_names.size()) {
        return _class_names.at(index);
    }
    else {
        return "";
    }
}

int TJ2NetworkTables::get_index(int obj_id) {
    return obj_id >> 16;
}


std::vector<std::string> TJ2NetworkTables::load_label_names(const std::string& path) {
    // load class names
    std::vector<std::string> class_names;
    std::ifstream infile(path);
    if (infile.is_open()) {
        std::string line;
        while (getline (infile,line)) {
            class_names.emplace_back(line);
        }
        infile.close();
    }
    else {
        std::cerr << "Error loading the class names!\n";
    }

    return class_names;
}


void TJ2NetworkTables::publish_odom()
{
    ros::Time recv_time = ros::Time::now();
    vector<double> odom_value = get_double_array(_odom_entry);
    
    if (odom_value.size() != 7) {
        ROS_WARN_THROTTLE(1.0, "Received invalid size for odom: %lu. Ignoring.", odom_value.size());
        return;
    }

    // index 0 is timestamp
    double x = odom_value[1];
    double y = odom_value[2];
    double t = odom_value[3];
    double vx = odom_value[4];
    double vy = odom_value[5];
    double vt = odom_value[6];

    if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(t) && std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vt))) {
        ROS_WARN_THROTTLE(1.0, "Odometry values are nan or inf");
        return;
    }

    tf2::Quaternion quat;
    quat.setRPY(0, 0, t);

    geometry_msgs::Quaternion msg_quat = tf2::toMsg(quat);

    _odom_msg.header.stamp = recv_time;
    _odom_msg.pose.pose.position.x = x;
    _odom_msg.pose.pose.position.y = y;
    _odom_msg.pose.pose.position.z = 0.0;
    _odom_msg.pose.pose.orientation = msg_quat;

    _odom_msg.twist.twist.linear.x = vx;
    _odom_msg.twist.twist.linear.y = vy;
    _odom_msg.twist.twist.angular.z = vt;

    if (_publish_odom_tf)
    {
        geometry_msgs::TransformStamped tf_stamped;
        tf_stamped.header.stamp = recv_time;
        tf_stamped.header.frame_id = _odom_frame;
        tf_stamped.child_frame_id = _base_frame;
        tf_stamped.transform.translation.x = x;
        tf_stamped.transform.translation.y = y;
        tf_stamped.transform.translation.z = 0.0;
        tf_stamped.transform.rotation = msg_quat;

        _tf_broadcaster.sendTransform(tf_stamped);
    }
    
    _odom_pub.publish(_odom_msg);
}

void TJ2NetworkTables::publish_imu()
{
    ros::Time recv_time = ros::Time::now();
    vector<double> imu_value = get_double_array(_imu_entry);
    
    if (imu_value.size() != 7) {
        ROS_WARN_THROTTLE(1.0, "Received invalid size for imu: %lu. Ignoring.", imu_value.size());
        return;
    }

    // index 0 is timestamp
    double tx = imu_value[1];
    double ty = imu_value[2];
    double tz = imu_value[3];
    double vz = imu_value[4];
    double ax = imu_value[5];
    double ay = imu_value[6];

    if (!(std::isfinite(tx) &&
          std::isfinite(ty) &&
          std::isfinite(tz) &&
          std::isfinite(vz) &&
          std::isfinite(ax) &&
          std::isfinite(ay)
        )) {
        ROS_WARN_THROTTLE(1.0, "A value for the IMU is nan or inf");
        return;
    }

    _imu_msg.header.stamp = recv_time;

    tf2::Quaternion quat;
    quat.setRPY(tx, ty, tz);
    _imu_msg.orientation = tf2::toMsg(quat);
    _imu_msg.linear_acceleration.x = ax;
    _imu_msg.linear_acceleration.y = ay;
    _imu_msg.angular_velocity.z = vz;  

    _imu_pub.publish(_imu_msg);
}

void TJ2NetworkTables::publish_detection(string name, int index, geometry_msgs::PoseStamped pose)
{
    geometry_msgs::TransformStamped transform;
    try {
        transform = _tf_buffer.lookupTransform(_base_frame, pose.header.frame_id, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "Failed to transform object to base frame: %s", ex.what());
        return;
    }

    geometry_msgs::PoseStamped base_pose;
    tf2::doTransform(pose, base_pose, transform);

    string key = _base_key + "detections/" + name + "/" + to_string(index);
    nt::SetEntryValue(nt::GetEntry(_nt, key + "/position/x"), nt::Value::MakeDouble(base_pose.pose.position.x));
    nt::SetEntryValue(nt::GetEntry(_nt, key + "/position/y"), nt::Value::MakeDouble(base_pose.pose.position.y));
    nt::SetEntryValue(nt::GetEntry(_nt, key + "/position/z"), nt::Value::MakeDouble(base_pose.pose.position.z));
    nt::SetEntryValue(nt::GetEntry(_nt, key + "/orientation/w"), nt::Value::MakeDouble(base_pose.pose.orientation.w));
    nt::SetEntryValue(nt::GetEntry(_nt, key + "/orientation/x"), nt::Value::MakeDouble(base_pose.pose.orientation.x));
    nt::SetEntryValue(nt::GetEntry(_nt, key + "/orientation/y"), nt::Value::MakeDouble(base_pose.pose.orientation.y));
    nt::SetEntryValue(nt::GetEntry(_nt, key + "/orientation/z"), nt::Value::MakeDouble(base_pose.pose.orientation.z));
}

void TJ2NetworkTables::publish_detection_count(string name, int count)
{
    nt::SetEntryValue(nt::GetEntry(_nt, _base_key + "detections/" + name + "/count"), nt::Value::MakeDouble(count));
    nt::SetEntryValue(nt::GetEntry(_nt, _base_key + "detections/" + name + "/update"), nt::Value::MakeDouble(get_time()));
}


// -----
// NT helpers
// -----

vector<double> TJ2NetworkTables::get_double_array(NT_Entry entry)
{
    auto value = nt::GetEntryValue(entry);
    vector<double> result;
    if (value == nullptr) {
        ROS_WARN_THROTTLE(1.0, "NT entry is NULL. Expected a double array!");
        return result;
    }
    else if (!value->IsDoubleArray()) {
        ROS_WARN_THROTTLE(1.0, "NT entry is not a double array as expected! Got type %d", value->type());
        return result;
    }
    else {
        result = value->GetDoubleArray();
        return result;
    }
}
vector<string> TJ2NetworkTables::get_string_array(NT_Entry entry)
{
    auto value = nt::GetEntryValue(entry);
    vector<string> result;
    if (value == nullptr) {
        ROS_WARN_THROTTLE(1.0, "NT entry is NULL. Expected a string array!");
        return result;
    }
    else if (!value->IsStringArray()) {
        ROS_WARN_THROTTLE(1.0, "NT entry is not a string array as expected! Got type %d", value->type());
        return result;
    }
    else {
        result = value->GetStringArray();
        return result;
    }
}


double TJ2NetworkTables::get_double(NT_Entry entry, double default_value)
{
    auto value = nt::GetEntryValue(entry);
    if (value && value->IsDouble()) {
        return value->GetDouble();
    }
    else {
        return default_value;
    }
}

bool TJ2NetworkTables::get_boolean(NT_Entry entry, bool default_value)
{
    auto value = nt::GetEntryValue(entry);
    if (value && value->IsBoolean()) {
        return value->GetBoolean();
    }
    else {
        return default_value;
    }
}

string TJ2NetworkTables::get_string(NT_Entry entry, string default_value)
{
    auto value = nt::GetEntryValue(entry);
    if (value && value->IsString()) {
        return value->GetString();
    }
    else {
        return default_value;
    }
}

NT_Entry TJ2NetworkTables::get_entry(string path) {
    return nt::GetEntry(_nt, _base_key + path);
}

// ---
// Main loop methods
// ---

void TJ2NetworkTables::loop()
{
    publish_odom();
    publish_imu();
    publish_cmd_vel();
    publish_robot_global_pose();
    ros::Duration odom_duration = ros::Time::now() - _prev_odom_timestamp;
    if (odom_duration > _odom_timeout) {
        ROS_WARN_THROTTLE(1.0, "No odom received for %f seconds", odom_duration.toSec());
    }
}

int TJ2NetworkTables::run()
{
    ros::Rate clock_rate(1.0 / _update_interval);  // Hz

    int exit_code = 0;
    while (ros::ok())
    {
        // let ROS process any events
        ros::spinOnce();
        clock_rate.sleep();

        try {
            loop();
        }
        catch (exception& e) {
            ROS_ERROR_STREAM("Exception in main loop: " << e.what());
            exit_code = 1;
            break;
        }
    }
    return exit_code;
}
