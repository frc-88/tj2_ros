// Required headers
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <ntcore.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <ros_msg_parser/ros_parser.hpp>
#include <base64_utils.h>
#include <jsoncpp/json/json.h>

bool jsonToRosMessage(const std::string& json_str, topic_tools::ShapeShifter& message, const RosIntrospection::Parser& parser);

// Subscribe to NT and publish to ROS
class NtPublisherBridge {
public:
  NtPublisherBridge(ros::NodeHandle& nh, std::shared_ptr<NetworkTable> nt_table, const std::vector<std::string>& nt_keys, unsigned int queue_size);
  void process_nt_entry(const std::string& nt_key, const std::string& base64_msg);

private:
    ros::NodeHandle nh_;
    std::shared_ptr<NetworkTable> nt_table_;
    RosIntrospection::Parser parser_;
    unsigned int queue_size_;
    std::map<std::string, ros::Publisher> publishers_;
};
