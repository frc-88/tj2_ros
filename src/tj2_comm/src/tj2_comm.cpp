
#include "tj2_comm.h"

TJ2Comm::TJ2Comm(ros::NodeHandle* nodehandle) :
    nh(*nodehandle), _tf_listener(_tf_buffer)
{
    ros::param::param<double>("~publish_rate", _publish_rate, 30.0);

    double ping_interval;
    ros::param::param<double>("~ping_interval", ping_interval, 0.5);

    double odom_timeout;
    ros::param::param<double>("~odom_timeout", odom_timeout, 0.1);
    _odom_timeout = ros::Duration(odom_timeout);

    ros::param::param<bool>("~publish_odom_tf", _publish_odom_tf, true);

    update_compact_ids();

    tf_compact_pub = nh.advertise<tf2_msgs::TFMessage>("/tf_compact", 50);
    ping_send_pub = nh.advertise<std_msgs::Float64>("ping_send", 1);
    ping_pub = nh.advertise<std_msgs::Float64>("ping", 10);

    ping_return_sub = nh.subscribe<std_msgs::Float64>("ping_return", 1, &TJ2Comm::ping_return_callback, this);

    _ping_timer = nh.createTimer(ros::Duration(ping_interval), &TJ2Comm::ping_timer_callback, this);

    ROS_INFO("tj2_comm is ready");
}

// ---
// Sub callbacks
// ---

void TJ2Comm::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    ros::Duration delta_time = ros::Time::now() - msg->header.stamp;
    if (delta_time > _odom_timeout) {
        ROS_WARN_THROTTLE(1.0, "Odometry is out of sync by %f seconds", delta_time.toSec());
        return;
    }
    _last_odom_time = msg->header.stamp;

    if (_publish_odom_tf)
    {
        geometry_msgs::TransformStamped tf_stamped;
        tf_stamped.header.stamp = msg->header.stamp;
        tf_stamped.header.frame_id = msg->header.frame_id;
        tf_stamped.child_frame_id = msg->child_frame_id;
        tf_stamped.transform.translation.x = msg->pose.pose.position.x;
        tf_stamped.transform.translation.y = msg->pose.pose.position.y;
        tf_stamped.transform.translation.z = msg->pose.pose.position.z;
        tf_stamped.transform.rotation.w = msg->pose.pose.orientation.w;
        tf_stamped.transform.rotation.x = msg->pose.pose.orientation.x;
        tf_stamped.transform.rotation.y = msg->pose.pose.orientation.y;
        tf_stamped.transform.rotation.z = msg->pose.pose.orientation.z;
        _tf_broadcaster.sendTransform(tf_stamped);
    }
}

void TJ2Comm::ping_return_callback(const std_msgs::Float64ConstPtr& msg)
{
    std_msgs::Float64 ping;
    ping.data = get_time() - msg->data;
    ping_pub.publish(ping);
}

// ---
// Timer callbacks
// ---

void TJ2Comm::ping_timer_callback(const ros::TimerEvent& event)
{
    std_msgs::Float64 msg;
    msg.data = get_time();
    ping_send_pub.publish(msg);
}

// ---
// Helpers
// ---

void TJ2Comm::update_compact_ids()
{
    XmlRpc::XmlRpcValue compact_ids_param;
    string key;
    if (!ros::param::search("tf_compact_ids", key)) {
        throw std::runtime_error("Failed to find tf_compact_ids parameter");
    }
    nh.getParam(key, compact_ids_param);

    if (compact_ids_param.getType() != XmlRpc::XmlRpcValue::Type::TypeArray ||
        compact_ids_param.size() == 0) {
        throw std::runtime_error("tf_compact_ids is the wrong type or size");
    }

    for (int index = 0; index < compact_ids_param.size(); index++) {
        if (compact_ids_param[index].getType() != XmlRpc::XmlRpcValue::TypeArray) {
            throw std::runtime_error("tf_compact_ids element is not a list");
        }
        vector<string> pair;
        if (compact_ids_param[index].size() != 2) {
            throw std::runtime_error("tf_compact_ids element is not size 2");
        }
        for (int pair_index = 0; pair_index < compact_ids_param[index].size(); pair_index++) {
            if (compact_ids_param[index][pair_index].getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR_STREAM("tf_compact_ids element is not a string: " << compact_ids_param[index][pair_index]);
                throw std::runtime_error("tf_compact_ids element is not a string");
            }
            pair.emplace_back((string)compact_ids_param[index][pair_index]);
        }
        compact_ids.emplace_back(pair);
    }
}

double TJ2Comm::get_time() {
    return ros::Time::now().toSec();
}

void TJ2Comm::publish_compact_tf()
{
    geometry_msgs::TransformStamped transform;
    tf2_msgs::TFMessage compacted_tree;
    for (size_t index = 0; index < compact_ids.size(); index++)
    {
        string parent_frame_id = compact_ids[index][0];
        string child_frame_id = compact_ids[index][1];

        try {
            transform = _tf_buffer.lookupTransform(parent_frame_id, child_frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            continue;
        }

        compacted_tree.transforms.push_back(transform);
    }
    tf_compact_pub.publish(compacted_tree);
}

void TJ2Comm::check_odom()
{
    ros::Duration delta_time = ros::Time::now() - _last_odom_time;
    if (delta_time > _odom_timeout) {
        ROS_WARN_THROTTLE(1.0, "No odometry received for %f seconds", delta_time.toSec());
    }
}

int TJ2Comm::run()
{
    ros::Rate clock_rate(_publish_rate);  // Hz
    while (ros::ok())
    {
        clock_rate.sleep();
        publish_compact_tf();
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_comm");
    ros::NodeHandle nh;
    TJ2Comm node(&nh);
    return node.run();
}
