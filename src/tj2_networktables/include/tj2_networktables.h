#pragma once

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include "ros_type_introspection/ros_introspection.hpp"
#include "ros/topic_manager.h"

#include "ntcore.h"
#include "networktables/EntryListenerFlags.h"
#include <llvm/StringRef.h>

#include <boost/assign/list_of.hpp>
#include <boost/unordered_map.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>

using namespace std;


struct TopicInfo {
    string topic_name;
    int queue_size;
};
typedef struct TopicInfo TopicInfo_t;


template<typename M>
inline ros::SerializedMessage serializeMessage(const M& message) {
    return message;
}

class TJ2NetworkTables
{
private:
    ros::NodeHandle nh;  // ROS node handle
    
    // Parameters
    string _nt_server;
    int _nt_port;
    double _update_interval;
    vector<vector<string>> compact_ids;
    set<string> _send_topics;
    map<string, TopicInfo_t> _recv_topics;
    
    // Members
    NT_Inst _nt;
    map<string, ros::Subscriber> _subscribers;
    map<string, ros::Publisher> _publishers;
    map<string, NT_Entry> _nt_subscribers;
    map<string, NT_Entry> _nt_publishers;
    topic_tools::ShapeShifter shape_shifter;
    
    // Sub callbacks
    void send_topic_callback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string &topic_name);
    void recv_topic_callback(const nt::EntryNotification& notification);

    // Helpers
    set<string> get_topics(string param_name);
    map<string, TopicInfo_t> get_topic_map(string param_name);
    void subscribe_to_send_topics(set<string> topic_names);
    void advertise_on_recv_topics(map<string, TopicInfo_t> topic_info);
    void setup_nt_connection(string _nt_server, unsigned int _nt_port);
    size_t get_entry_raw(NT_Entry entry, uint8_t** buffer);

public:
    TJ2NetworkTables(ros::NodeHandle* nodehandle);
    int run();
};

