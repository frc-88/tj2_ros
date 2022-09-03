#include "tj2_networktables.h"

TJ2NetworkTables::TJ2NetworkTables(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
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

    _imu_pub = nh.advertise<tj2_networktables::NavX>("imu", 50);
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
    
    for (size_t index = 0; index < _joint_names.size(); index++) {
        add_joint(_joint_names.at(index));
    }

    _match_time_pub = nh.advertise<std_msgs::Float64>("match_time", 10);
    _autonomous_pub = nh.advertise<std_msgs::Bool>("is_autonomous", 10);
    _team_color_pub = nh.advertise<std_msgs::String>("team_color", 10);

    _pose_estimate_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
    _pose_reset_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("reset_pose", 10);  // for distiguishing between ROS and RIO requested resets

    _waypoints_action_client = new actionlib::SimpleActionClient<tj2_waypoints::FollowPathAction>("follow_path", true);

    _twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 50, &TJ2NetworkTables::twist_callback, this);
    _nt_passthrough_sub = nh.subscribe<tj2_networktables::NTEntry>("nt_passthrough", 50, &TJ2NetworkTables::nt_passthrough_callback, this);
    _waypoints_sub = nh.subscribe<tj2_waypoints::WaypointArray>("waypoints", 50, &TJ2NetworkTables::waypoints_callback, this);
    if (_class_names.empty()) {
        ROS_ERROR("Error loading class names! Not broadcasting detections");
    }
    else {
        _detections_sub = nh.subscribe<vision_msgs::Detection3DArray>("detections", 50, &TJ2NetworkTables::detections_callback, this);
    }

    _prev_twist_timestamp = ros::Time(0);
    _twist_cmd_vx = 0.0;
    _twist_cmd_vy = 0.0;
    _twist_cmd_vt = 0.0;

    _currentGoalStatus = INVALID;

    _odom_reset_srv = nh.advertiseService("odom_reset_service", &TJ2NetworkTables::odom_reset_callback, this);

    _ping_pub = nh.advertise<std_msgs::Float64>("ping", 50);
    _ping_timer = nh.createTimer(ros::Duration(0.5), &TJ2NetworkTables::ping_timer_callback, this);
    _joint_timer = nh.createTimer(ros::Duration(0.1), &TJ2NetworkTables::joint_timer_callback, this);

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

    _odom_x_entry = nt::GetEntry(_nt, _base_key + "odom/x");
    _odom_y_entry = nt::GetEntry(_nt, _base_key + "odom/y");
    _odom_t_entry = nt::GetEntry(_nt, _base_key + "odom/t");
    _odom_vx_entry = nt::GetEntry(_nt, _base_key + "odom/vx");
    _odom_vy_entry = nt::GetEntry(_nt, _base_key + "odom/vy");
    _odom_vt_entry = nt::GetEntry(_nt, _base_key + "odom/vt");
    _odom_update_entry = nt::GetEntry(_nt, _base_key + "odom/update");
    nt::AddEntryListener(_odom_update_entry, boost::bind(&TJ2NetworkTables::odom_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _ping_entry = nt::GetEntry(_nt, _base_key + "ping");
    _ping_return_entry = nt::GetEntry(_nt, _base_key + "ping_return");
    nt::AddEntryListener(_ping_return_entry, boost::bind(&TJ2NetworkTables::ping_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _set_odom_x_entry = nt::GetEntry(_nt, _base_key + "reset_odom/x");
    _set_odom_y_entry = nt::GetEntry(_nt, _base_key + "reset_odom/y");
    _set_odom_t_entry = nt::GetEntry(_nt, _base_key + "reset_odom/t");
    _set_odom_update_entry = nt::GetEntry(_nt, _base_key + "reset_odom/update");

    _imu_tx_entry = nt::GetEntry(_nt, _base_key + "imu/tx");
    _imu_ty_entry = nt::GetEntry(_nt, _base_key + "imu/ty");
    _imu_tz_entry = nt::GetEntry(_nt, _base_key + "imu/tz");
    _imu_vx_entry = nt::GetEntry(_nt, _base_key + "imu/vx");
    _imu_vy_entry = nt::GetEntry(_nt, _base_key + "imu/vy");
    _imu_vz_entry = nt::GetEntry(_nt, _base_key + "imu/vz");
    _imu_ax_entry = nt::GetEntry(_nt, _base_key + "imu/ax");
    _imu_ay_entry = nt::GetEntry(_nt, _base_key + "imu/ay");
    _imu_update_entry = nt::GetEntry(_nt, _base_key + "imu/update");
    nt::AddEntryListener(_imu_update_entry, boost::bind(&TJ2NetworkTables::imu_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _cmd_vel_x_entry = nt::GetEntry(_nt, _base_key + "cmd_vel/x");
    _cmd_vel_y_entry = nt::GetEntry(_nt, _base_key + "cmd_vel/y");
    _cmd_vel_t_entry = nt::GetEntry(_nt, _base_key + "cmd_vel/t");
    _cmd_vel_update_entry = nt::GetEntry(_nt, _base_key + "cmd_vel/update");

    _goal_status_entry = nt::GetEntry(_nt, _base_key + "goal_status/status");
    _goal_status_update_entry = nt::GetEntry(_nt, _base_key + "goal_status/update");

    _global_x_entry = nt::GetEntry(_nt, _base_key + "global/x");
    _global_y_entry = nt::GetEntry(_nt, _base_key + "global/y");
    _global_t_entry = nt::GetEntry(_nt, _base_key + "global/t");
    _global_update_entry = nt::GetEntry(_nt, _base_key + "global/update");

    _match_time_entry = nt::GetEntry(_nt, _base_key + "match/time");
    _is_auto_entry = nt::GetEntry(_nt, _base_key + "match/is_auto");
    _team_color_entry = nt::GetEntry(_nt, _base_key + "match/team_color");
    _match_update_entry = nt::GetEntry(_nt, _base_key + "match/update");
    nt::AddEntryListener(_match_update_entry, boost::bind(&TJ2NetworkTables::match_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _pose_est_x_entry = nt::GetEntry(_nt, _base_key + "pose_est/x");
    _pose_est_y_entry = nt::GetEntry(_nt, _base_key + "pose_est/y");
    _pose_est_t_entry = nt::GetEntry(_nt, _base_key + "pose_est/t");
    _pose_est_update_entry = nt::GetEntry(_nt, _base_key + "pose_est/update");
    nt::AddEntryListener(_pose_est_update_entry, boost::bind(&TJ2NetworkTables::pose_estimate_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _exec_waypoint_plan_entry = nt::GetEntry(_nt, _base_key + "plan/exec");
    _exec_waypoint_plan_update_entry = nt::GetEntry(_nt, _base_key + "plan/exec_update");
    _reset_waypoint_plan_entry = nt::GetEntry(_nt, _base_key + "plan/reset");
    _cancel_waypoint_plan_entry = nt::GetEntry(_nt, _base_key + "plan/cancel");
    nt::AddEntryListener(_exec_waypoint_plan_update_entry, boost::bind(&TJ2NetworkTables::exec_waypoint_plan_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);
    nt::AddEntryListener(_reset_waypoint_plan_entry, boost::bind(&TJ2NetworkTables::reset_waypoint_plan_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);
    nt::AddEntryListener(_cancel_waypoint_plan_entry, boost::bind(&TJ2NetworkTables::cancel_waypoint_plan_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

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

    nt::SetEntryValue(_cmd_vel_x_entry, nt::Value::MakeDouble(_twist_cmd_vx));
    nt::SetEntryValue(_cmd_vel_y_entry, nt::Value::MakeDouble(_twist_cmd_vy));
    nt::SetEntryValue(_cmd_vel_t_entry, nt::Value::MakeDouble(_twist_cmd_vt));
    nt::SetEntryValue(_cmd_vel_update_entry, nt::Value::MakeDouble(get_time()));
}

void TJ2NetworkTables::publish_goal_status()
{
    actionlib::SimpleClientGoalState state = _waypoints_action_client->getState();

    GoalStatus currentPollStatus;

    // Possible states:  PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
    if (state.isDone()) {
        // RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, or LOST.
        if (state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
            currentPollStatus = GoalStatus::FINISHED;
        }
        else {
            currentPollStatus = GoalStatus::FAILED;
        }
    }
    else {
        // PENDING or ACTIVE
        currentPollStatus = GoalStatus::RUNNING;
    }

    if (currentPollStatus != _prevPollStatus) {
        ROS_INFO("Current goal status changed to: %d", _currentGoalStatus);
        _prevPollStatus = currentPollStatus;
        _currentGoalStatus = currentPollStatus;
        nt::SetEntryValue(_goal_status_entry, nt::Value::MakeDouble((double)_currentGoalStatus));
        nt::SetEntryValue(_goal_status_update_entry, nt::Value::MakeDouble(get_time()));
    }
}

void TJ2NetworkTables::publish_robot_global_pose()
{
    tf::StampedTransform transform;
    try {
        _tf_listener.lookupTransform(_map_frame, _base_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        return;
    }

    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double theta = tf::getYaw(transform.getRotation()); 
    
    nt::SetEntryValue(_global_x_entry, nt::Value::MakeDouble(x));
    nt::SetEntryValue(_global_y_entry, nt::Value::MakeDouble(y));
    nt::SetEntryValue(_global_t_entry, nt::Value::MakeDouble(theta));
    nt::SetEntryValue(_global_update_entry, nt::Value::MakeDouble(get_time()));
}

void TJ2NetworkTables::publish_joints()
{
    for (size_t index = 0; index < _joint_names.size(); index++) {
        joint_callback(index);
    }
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
    _twist_cmd_vx = vx;
    _twist_cmd_vy = vy;
    _twist_cmd_vt = vt;
}

void TJ2NetworkTables::nt_passthrough_callback(const tj2_networktables::NTEntryConstPtr& msg)
{
    NT_Entry entry = nt::GetEntry(_nt, _base_key + msg->path);
    nt::SetEntryValue(entry, nt::Value::MakeDouble(msg->value));
}

void TJ2NetworkTables::waypoints_callback(const tj2_waypoints::WaypointArrayConstPtr& msg)
{
    for (size_t index = 0; index < msg->waypoints.size(); index++)
    {
        string name = msg->waypoints.at(index).name;
        NT_Entry x_entry = nt::GetEntry(_nt, _base_key + "waypoints/" + name + "/x");
        NT_Entry y_entry = nt::GetEntry(_nt, _base_key + "waypoints/" + name + "/y");
        NT_Entry theta_entry = nt::GetEntry(_nt, _base_key + "waypoints/" + name + "/theta");

        geometry_msgs::Pose pose = msg->waypoints.at(index).pose;
        double x = pose.position.x;
        double y = pose.position.y;

        tf::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        nt::SetEntryValue(x_entry, nt::Value::MakeDouble(x));
        nt::SetEntryValue(y_entry, nt::Value::MakeDouble(y));
        nt::SetEntryValue(theta_entry, nt::Value::MakeDouble(yaw));
    }
}

void TJ2NetworkTables::detections_callback(const vision_msgs::Detection3DArrayConstPtr& msg)
{
    if (_class_names.empty()) {
        ROS_ERROR("No class names loaded! Not broadcasting detections");
        return;
    }

    for (size_t name_index = 0; name_index < _class_names.size(); name_index++) {
        double min_dist = NAN;
        string label = _class_names.at(name_index);
        geometry_msgs::Pose nearest_pose;
        int num_detections = 0;
        for (size_t index = 0; index < msg->detections.size(); index++) {
            vision_msgs::ObjectHypothesisWithPose hyp = msg->detections.at(index).results[0];
            string name = get_label(hyp.id);
            if (name == label) {
                num_detections++;
                geometry_msgs::Pose pose = hyp.pose.pose;
                double dist = sqrt(pose.position.x * pose.position.x + pose.position.y * pose.position.y + pose.position.z * pose.position.z);
                if (!std::isfinite(min_dist) || dist < min_dist) {
                    min_dist = dist;
                    nearest_pose = hyp.pose.pose;
                }
            }
        }
        nt::SetEntryValue(nt::GetEntry(_nt, _base_key + "detections/" + label + "/count"), nt::Value::MakeDouble(num_detections));
        nt::SetEntryValue(nt::GetEntry(_nt, _base_key + "detections/" + label + "/update"), nt::Value::MakeDouble(get_time()));
        
        if (!std::isfinite(min_dist)) {
            continue;
        }

        double nearest_x = nearest_pose.position.x;
        double nearest_y = nearest_pose.position.y;
        double nearest_z = nearest_pose.position.z;

        nt::SetEntryValue(nt::GetEntry(_nt, _base_key + "detections/" + label + "/x"), nt::Value::MakeDouble(nearest_x));
        nt::SetEntryValue(nt::GetEntry(_nt, _base_key + "detections/" + label + "/y"), nt::Value::MakeDouble(nearest_y));
        nt::SetEntryValue(nt::GetEntry(_nt, _base_key + "detections/" + label + "/z"), nt::Value::MakeDouble(nearest_z));
    }
}


void TJ2NetworkTables::joint_command_callback(const std_msgs::Float64ConstPtr& msg, string joint_name, int joint_index)
{
    NT_Entry joint_entry = nt::GetEntry(_nt, _base_key + "joints/commands/value/" + std::to_string(joint_index));
    NT_Entry update_entry = nt::GetEntry(_nt, _base_key + "joints/commands/update/" + std::to_string(joint_index));

    nt::SetEntryValue(joint_entry, nt::Value::MakeDouble(msg->data));
    nt::SetEntryValue(update_entry, nt::Value::MakeDouble(get_time()));
}


// ---
// Service callbacks
// ---
bool TJ2NetworkTables::odom_reset_callback(tj2_networktables::OdomReset::Request &req, tj2_networktables::OdomReset::Response &resp)
{
    nt::SetEntryValue(_set_odom_x_entry, nt::Value::MakeDouble(req.x));
    nt::SetEntryValue(_set_odom_y_entry, nt::Value::MakeDouble(req.y));
    nt::SetEntryValue(_set_odom_t_entry, nt::Value::MakeDouble(req.t));
    nt::SetEntryValue(_set_odom_update_entry, nt::Value::MakeDouble(get_time()));
    ROS_INFO("Resetting odometry to x: %0.3f, y: %0.3f, theta: %0.3f", req.x, req.y, req.t);
    resp.resp = true;
    return true;
}

// ---
// NT callbacks
// ---

void TJ2NetworkTables::odom_callback(const nt::EntryNotification& event)
{
    // publish_odom();
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
    // publish_imu();
    _prev_imu_timestamp = ros::Time::now();
}

void TJ2NetworkTables::joint_callback(size_t joint_index)
{
    NT_Entry joint_entry = nt::GetEntry(_nt, _base_key + "joints/" + std::to_string(joint_index));
    double joint_position = get_double(joint_entry, NAN);
    if (joint_index >= _raw_joint_msgs->size()) {
        ROS_WARN("Invalid joint index received: %ld. Valid range is 0..%ld. (Joint value was %f)", joint_index, _raw_joint_msgs->size() - 1, joint_position);
        return;
    }
    if (!std::isfinite(joint_position)) {
        ROS_WARN_THROTTLE(1.0, "Joint position for index %ld is nan or inf", joint_index);
        return;
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

    double x = get_double(_pose_est_x_entry, NAN);
    double y = get_double(_pose_est_y_entry, NAN);
    double theta = get_double(_pose_est_t_entry, NAN);

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

void TJ2NetworkTables::create_waypoint(size_t index)
{
    tj2_waypoints::Waypoint waypoint = make_waypoint_from_nt(index);
    double x = get_double(_waypoint_goal_x_entry, NAN);
    double y = get_double(_waypoint_goal_y_entry, NAN);
    double theta = get_double(_waypoint_goal_t_entry, NAN);
    string waypoint_name = get_string(_waypoint_goal_name_entry, "");
    waypoint.name = waypoint_name;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, theta);

    geometry_msgs::Quaternion msg_quat = tf2::toMsg(quat);

    waypoint.pose.position.x = x;
    waypoint.pose.position.y = y;
    waypoint.pose.orientation = msg_quat;

    add_waypoint(waypoint);
}

void TJ2NetworkTables::exec_waypoint_plan_callback(const nt::EntryNotification& event)
{
    ROS_INFO("Received execute plan command");
    
    size_t num_waypoints = (size_t)get_double(_exec_waypoint_plan_entry, 0.0);
    for (size_t index = 0; index < num_waypoints; index++) {
        create_waypoint(index);
    }

    if (num_waypoints != _waypoints.waypoints.size()) {
        ROS_ERROR("The reported number of waypoints in the plan does match the number received! %ld != %ld Canceling plan", num_waypoints, _waypoints.waypoints.size());
        set_goal_status(GoalStatus::FAILED);
    }
    else {
        send_waypoints();
    }
    reset_waypoints();
}

void TJ2NetworkTables::reset_waypoint_plan_callback(const nt::EntryNotification& event)
{
    ROS_INFO("Received reset plan command");
    reset_waypoints();
}

void TJ2NetworkTables::cancel_waypoint_plan_callback(const nt::EntryNotification& event)
{
    ROS_INFO("Received cancel plan command");
    cancel_waypoint_goal();
}

// ---
// Timer callbacks
// ---
void TJ2NetworkTables::ping_timer_callback(const ros::TimerEvent& event)
{
    nt::SetEntryValue(_ping_entry, nt::Value::MakeDouble(get_time()));
}

void TJ2NetworkTables::joint_timer_callback(const ros::TimerEvent& event)
{
    publish_joints();
}


// ---
// Other helpers
// ---

void TJ2NetworkTables::add_joint(string name)
{
    size_t joint_index = _raw_joint_pubs->size();
    ROS_INFO("Publishing to joint topic: %s. Index: %ld", name.c_str(), joint_index);
    _raw_joint_pubs->push_back(nh.advertise<std_msgs::Float64>(name, 50));
    _raw_joint_msgs->push_back(new std_msgs::Float64);

    _raw_joint_subs->push_back(
        nh.subscribe<std_msgs::Float64>(
            "joint_command/" + name, 50,
            boost::bind(&TJ2NetworkTables::joint_command_callback, this, _1, name, joint_index)
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

void TJ2NetworkTables::add_waypoint(tj2_waypoints::Waypoint waypoint)
{
    _waypoints.waypoints.insert(_waypoints.waypoints.end(), waypoint);
    ROS_INFO("Received a waypoint: %s. pose: x=%0.4f, y=%0.4f. is_continuous: %d, ignore_orientation: %d, ignore_obstacles: %d, ignore_walls: %d, interruptable_by: %s, timeout: %f",
        waypoint.name.c_str(),
        waypoint.pose.position.x,
        waypoint.pose.position.y,
        waypoint.is_continuous,
        waypoint.ignore_orientation,
        waypoint.ignore_obstacles,
        waypoint.ignore_walls,
        waypoint.interruptable_by.c_str(),
        waypoint.timeout.toSec()
    );
}

tj2_waypoints::Waypoint TJ2NetworkTables::make_waypoint_from_nt(size_t index)
{
    string str_index = std::to_string(index);
    _waypoint_is_continuous_entry = nt::GetEntry(_nt, _base_key + "goal/" + str_index + "/is_continuous");
    _waypoint_ignore_orientation_entry = nt::GetEntry(_nt, _base_key + "goal/" + str_index + "/ignore_orientation");
    _waypoint_intermediate_tolerance_entry = nt::GetEntry(_nt, _base_key + "goal/" + str_index + "/intermediate_tolerance");
    _waypoint_ignore_obstacles_entry = nt::GetEntry(_nt, _base_key + "goal/" + str_index + "/ignore_obstacles");
    _waypoint_ignore_walls_entry = nt::GetEntry(_nt, _base_key + "goal/" + str_index + "/ignore_walls");
    _waypoint_interruptable_by_entry = nt::GetEntry(_nt, _base_key + "goal/" + str_index + "/interruptable_by");
    _waypoint_timeout_entry = nt::GetEntry(_nt, _base_key + "goal/" + str_index + "/timeout");
    _waypoint_goal_x_entry = nt::GetEntry(_nt, _base_key + "goal/" + str_index + "/x");
    _waypoint_goal_y_entry = nt::GetEntry(_nt, _base_key + "goal/" + str_index + "/y");
    _waypoint_goal_t_entry = nt::GetEntry(_nt, _base_key + "goal/" + str_index + "/t");
    _waypoint_goal_name_entry = nt::GetEntry(_nt, _base_key + "goal/" + str_index + "/name");

    tj2_waypoints::Waypoint waypoint;
    waypoint.is_continuous = get_boolean(_waypoint_is_continuous_entry, false);
    waypoint.ignore_orientation = get_boolean(_waypoint_ignore_orientation_entry, false);
    waypoint.intermediate_tolerance = get_double(_waypoint_intermediate_tolerance_entry, 0.1);
    waypoint.ignore_obstacles = get_boolean(_waypoint_ignore_obstacles_entry, false);
    waypoint.ignore_walls = get_boolean(_waypoint_ignore_walls_entry, false);
    waypoint.interruptable_by = get_string(_waypoint_interruptable_by_entry, "");
    waypoint.timeout = ros::Duration(get_double(_waypoint_timeout_entry, 0.0));
    return waypoint;
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
    double x = get_double(_odom_x_entry, NAN);
    double y = get_double(_odom_y_entry, NAN);
    double t = get_double(_odom_t_entry, NAN);
    double vx = get_double(_odom_vx_entry, NAN);
    double vy = get_double(_odom_vy_entry, NAN);
    double vt = get_double(_odom_vt_entry, NAN);
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
    double tx = get_double(_imu_tx_entry, NAN);
    double ty = get_double(_imu_ty_entry, NAN);
    double tz = get_double(_imu_tz_entry, NAN);
    double vx = get_double(_imu_vx_entry, NAN);
    double vy = get_double(_imu_vy_entry, NAN);
    double vz = get_double(_imu_vz_entry, NAN);
    double ax = get_double(_imu_ax_entry, NAN);
    double ay = get_double(_imu_ay_entry, NAN);

    if (!(std::isfinite(tx) &&
          std::isfinite(ty) &&
          std::isfinite(tz) &&
          std::isfinite(vx) &&
          std::isfinite(vy) &&
          std::isfinite(vz) &&
          std::isfinite(ax) &&
          std::isfinite(ay)
        )) {
        ROS_WARN_THROTTLE(1.0, "A value for the IMU is nan or inf");
    }

    _imu_msg.header.stamp = recv_time;

    tf2::Quaternion quat;
    quat.setRPY(tx, ty, tz);
    _imu_msg.orientation = tf2::toMsg(quat);
    _imu_msg.linear_acceleration.x = ax;
    _imu_msg.linear_acceleration.y = ay;
    _imu_msg.linear_acceleration.z = 0.0;
    _imu_msg.linear_velocity.x = vx;
    _imu_msg.linear_velocity.y = vy;
    _imu_msg.angular_velocity.z = vz;  

    _imu_pub.publish(_imu_msg);
}

// ---
// Waypoint control
// ---
void TJ2NetworkTables::set_goal_status(GoalStatus status)
{
    _currentGoalStatus = status;
}

void TJ2NetworkTables::send_waypoints()
{
    ROS_INFO("Sending waypoints");
    tj2_waypoints::FollowPathGoal goal;
    goal.waypoints = _waypoints;
    _waypoints_action_client->sendGoal(goal);
}

void TJ2NetworkTables::cancel_waypoint_goal()
{
    ROS_INFO("Canceling waypoint goal");
    _waypoints_action_client->cancelAllGoals();
    reset_waypoints();
}

void TJ2NetworkTables::reset_waypoints()
{
    _waypoints.waypoints.clear();
}


// -----
// NT helpers
// -----

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

// ---
// Main loop methods
// ---

void TJ2NetworkTables::loop()
{
    publish_odom();
    // publish_imu();
    publish_cmd_vel();
    publish_goal_status();
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_networktables");
    ros::NodeHandle nh;

    TJ2NetworkTables broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}