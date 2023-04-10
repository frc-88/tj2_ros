#include <ros/ros.h>
#include "nt_subscriber_bridge.h"
#include "nt_publisher_bridge.h"
#include <ntcore.h>

int main(int argc, char** argv) {
    // Initialize ROS and create a node handle
    ros::init(argc, argv, "tj2_networktables");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // Load NetworkTables mode parameter
    bool is_server_mode;
    private_nh.param("is_server", is_server_mode, false);

    // Load team number parameter
    std::string address;
    private_nh.param<std::string>("address", address, "");

    // Load update interval parameter
    double update_interval;
    private_nh.param("update_interval", update_interval, 0.02);

    // Load queue size parameter
    int queue_size;
    private_nh.param("queue_size", queue_size, 10);

    // Load NetworkTables port parameter
    int nt_port;
    private_nh.param("nt_port", nt_port, 1735);

    nt::NetworkTableInstance nt = nt::NetworkTableInstance::GetDefault();

    // Initialize NetworkTables
    if (is_server_mode) {
        nt.StartServer("tj2_networktables.ini", address.c_str(), nt_port);
    } else {
        nt.StartClient(address.c_str(), nt_port);
    }


    // Set NetworkTables update rate to update_interval
    nt.SetUpdateRate(update_interval);

    // Get a shared pointer to the desired NetworkTable
    auto nt_sub_table = nt.GetTable("sub");
    auto nt_pub_table = nt.GetTable("pub");

    // Load parameters for ROS topics and NetworkTables keys
    std::vector<std::string> ros_topics, nt_keys;
    private_nh.getParam("ros_topics", ros_topics);
    private_nh.getParam("nt_keys", nt_keys);

    // Create ROSNode and NTBridge instances
    NtSubscriberBridge subscriber_bridge(nh, nt_sub_table, ros_topics, queue_size);
    NtPublisherBridge publisher_bridge(nh, nt_pub_table, nt_keys, queue_size);

    // Spin the node
    ros::spin();

    // Clean up
    nt.StopServer();

    return 0;
}