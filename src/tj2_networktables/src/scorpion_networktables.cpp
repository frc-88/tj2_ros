#include "scorpion_networktables.h"

ScorpionNetworkTables::ScorpionNetworkTables(ros::NodeHandle* nodehandle) :
    TJ2NetworkTables(nodehandle)
{
    ros::param::param<string>("~tag_global_frame", _tag_global_frame, "field");

    _tag_global_entry = nt::GetEntry(_nt, _base_key + "tag_global");

    _tag_global_pose.resize(5);
    _tag_global_pose[0] = 0.0;
    _tag_global_pose[1] = 0.0;
    _tag_global_pose[2] = 0.0;
    _tag_global_pose[3] = 0.0;
    _tag_global_pose[4] = 0.0;

    ROS_INFO("scorpion_networktables init complete");
}

// ---
// Other helpers
// ---

void ScorpionNetworkTables::publish_tag_global_pose()
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

void ScorpionNetworkTables::loop()
{
    TJ2NetworkTables::loop();
    publish_tag_global_pose();
}
