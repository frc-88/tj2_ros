#include "diffyjr_networktables.h"

DiffyJrNetworkTables::DiffyJrNetworkTables(ros::NodeHandle* nodehandle) :
    TJ2NetworkTables(nodehandle)
{
    ros::param::param<string>("~tag_global_frame", _tag_global_frame, "field");

    _modules_pub = nh.advertise<tj2_interfaces::SwerveModule>("swerve_modules", 10);

    _num_modules = 0;

    _tag_global_pose.resize(5);
    _tag_global_pose[0] = 0.0;
    _tag_global_pose[1] = 0.0;
    _tag_global_pose[2] = 0.0;
    _tag_global_pose[3] = 0.0;
    _tag_global_pose[4] = 0.0;

    _module_num_entry = nt::GetEntry(_nt, _base_key + "modules/num");
    nt::AddEntryListener(_module_num_entry, boost::bind(&DiffyJrNetworkTables::module_num_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _tag_global_entry = nt::GetEntry(_nt, _base_key + "tag_global");

    ROS_INFO("diffyjr_networktables init complete");
}

// ---
// NT callbacks
// ---

void DiffyJrNetworkTables::module_num_callback(const nt::EntryNotification& event)
{
    _num_modules = (int)get_double(_module_num_entry, 0);
}

// ---
// Other helpers
// ---

void DiffyJrNetworkTables::publish_module()
{
    for (int index = 0; index < _num_modules; index++) {
        string module_index = std::to_string(index);
        tj2_interfaces::SwerveModule module_msg;
        tj2_interfaces::SwerveMotor lo_motor_msg;
        tj2_interfaces::SwerveMotor hi_motor_msg;
        module_msg.module_index = module_index;
        module_msg.wheel_velocity = get_double(get_entry("modules/" + module_index + "/wheel_velocity"), 0.0);
        module_msg.azimuth_velocity = get_double(get_entry("modules/" + module_index + "/azimuth_velocity"), 0.0);
        module_msg.azimuth_position = get_double(get_entry("modules/" + module_index + "/azimuth"), 0.0);
        module_msg.wheel_velocity_ref = get_double(get_entry("modules/" + module_index + "/wheel_velocity_ref"), 0.0);
        module_msg.azimuth_velocity_ref = get_double(get_entry("modules/" + module_index + "/azimuth_velocity_ref"), 0.0);
        module_msg.azimuth_position_ref = get_double(get_entry("modules/" + module_index + "/azimuth_ref"), 0.0);
        hi_motor_msg.voltage = get_double(get_entry("modules/" + module_index + "/hi_voltage"), 0.0);
        hi_motor_msg.voltage_ref = get_double(get_entry("modules/" + module_index + "/hi_voltage_ref"), 0.0);
        hi_motor_msg.velocity = get_double(get_entry("modules/" + module_index + "/hi_velocity"), 0.0);
        lo_motor_msg.voltage = get_double(get_entry("modules/" + module_index + "/lo_voltage"), 0.0);
        lo_motor_msg.voltage_ref = get_double(get_entry("modules/" + module_index + "/lo_voltage_ref"), 0.0);
        lo_motor_msg.velocity = get_double(get_entry("modules/" + module_index + "/lo_velocity"), 0.0);
        module_msg.motor_lo_0 = lo_motor_msg;
        module_msg.motor_hi_1 = hi_motor_msg;
        _modules_pub.publish(module_msg);
    }
}

void DiffyJrNetworkTables::publish_tag_global_pose()
{
    geometry_msgs::TransformStamped transform;
    try {
        transform = _tf_buffer.lookupTransform(_tag_global_frame, _base_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "Failed to publish robot's tag global pose: %s", ex.what());
        return;
    }

    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;
    double z = transform.transform.translation.z;

    tf2::Quaternion quat;
    tf2::convert(transform.transform.rotation, quat);
    tf2::Matrix3x3 m1(quat);
    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    
    _tag_global_pose[0] = get_time();
    _tag_global_pose[1] = x;
    _tag_global_pose[2] = y;
    _tag_global_pose[3] = z;
    _tag_global_pose[4] = yaw;

    nt::SetEntryValue(_tag_global_entry, nt::Value::MakeDoubleArray(_tag_global_pose));
}
// ---
// Main loop methods
// ---

void DiffyJrNetworkTables::loop()
{
    TJ2NetworkTables::loop();
    publish_module();
    publish_tag_global_pose();
}
