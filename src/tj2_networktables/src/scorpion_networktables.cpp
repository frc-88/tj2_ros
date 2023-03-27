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

    _nearest_cone_entry = nt::GetEntry(_nt, _base_key + "nearest_cone");
    _nearest_cone_pose.resize(8);
    _nearest_cone_pose[0] = 0.0;  // time
    _nearest_cone_pose[1] = 0.0;  // x
    _nearest_cone_pose[2] = 0.0;  // y
    _nearest_cone_pose[3] = 0.0;  // z
    _nearest_cone_pose[4] = 0.0;  // qx
    _nearest_cone_pose[5] = 0.0;  // qy
    _nearest_cone_pose[6] = 0.0;  // qz
    _nearest_cone_pose[7] = 0.0;  // qw

    _nearest_cone_sub = nh.subscribe<geometry_msgs::PoseStamped>("cones/nearest", 50, &ScorpionNetworkTables::nearest_cone_callback, this);

    ROS_INFO("scorpion_networktables init complete");
}

void ScorpionNetworkTables::nearest_cone_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    _nearest_cone_pose[0] = get_time();
    _nearest_cone_pose[1] = msg->pose.position.x;
    _nearest_cone_pose[2] = msg->pose.position.y;
    _nearest_cone_pose[3] = msg->pose.position.z;
    _nearest_cone_pose[4] = msg->pose.orientation.x;
    _nearest_cone_pose[5] = msg->pose.orientation.y;
    _nearest_cone_pose[6] = msg->pose.orientation.z;
    _nearest_cone_pose[7] = msg->pose.orientation.w;

    nt::SetEntryValue(_nearest_cone_entry, nt::Value::MakeDoubleArray(_nearest_cone_pose));
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
