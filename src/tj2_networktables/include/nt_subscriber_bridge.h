// Required headers
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <ntcore.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <ros_msg_parser/ros_parser.hpp>
#include <base64_utils.h>
#include <jsoncpp/json/json.h>

using RosIntrospection::Parser;
using RosIntrospection::ROSType;
using RosIntrospection::Variant;

std::string rosMessageToJson(ParsersCollection::DeserializedMsg* message, const RosIntrospection::Parser& parser);
Json::Value variantToJson(const RosIntrospection::Variant& var);

// Subscribe to ROS and publish to NT
class NtSubscriberBridge {
public:
  NtSubscriberBridge(ros::NodeHandle& nh, std::shared_ptr<NetworkTable> nt_table, const std::vector<std::string>& topics, unsigned int queue_size);
  void process_message(const std::string& topic, const topic_tools::ShapeShifter::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    std::shared_ptr<NetworkTable> nt_table_;
    RosMsgParser::ParsersCollection parsers_;
    unsigned int queue_size_;
    std::vector<ros::Subscriber> subscribers_;
};