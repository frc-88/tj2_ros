#pragma once

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include "ros_type_introspection/ros_introspection.hpp"

#include "ntcore.h"
#include "networktables/EntryListenerFlags.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>

using namespace std;

class TJ2NetworkTables
{
private:
    ros::NodeHandle nh;  // ROS node handle
    
    // Parameters
    int _nt_port;
    double _update_interval;
    vector<vector<string>> compact_ids;
    set<string> _send_topics;
    set<string> _recv_topics;
    
    // Members
    NT_Inst _nt;
    map<string, ros::Subscriber> _subscribers;
    map<string, ros::Publisher> _publishers;
    map<string, NT_Entry> _nt_subscribers;
    map<string, NT_Entry> _nt_publishers;
    
    // Sub callbacks
    void send_topic_callback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string &topic_name);

    // Helpers
    set<string> get_topics(string param_name);
    void subscribe_to_topics(set<string> topic_names);
    void setup_nt_server();

public:
    TJ2NetworkTables(ros::NodeHandle* nodehandle);
    int run();
};

