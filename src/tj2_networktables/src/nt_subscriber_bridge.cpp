#include "nt_subscriber_bridge.h"


std::string rosMessageToJson(ParsersCollection::DeserializedMsg* message, const RosIntrospection::Parser& parser) {
    Json::Value json_msg;
    // List of all those parsed fields that can be represented by a builtin value equal to "string".
    for (auto it : deserialized_msg->flat_msg.name) {
        const std::string& key = it.first.toStdString();
        const std::string& value = it.second;
        json_msg[key] = value;
    }
    // List of all those parsed fields that can be represented by a builtin value different from "string".
    for (auto it : deserialized_msg->flat_msg.name) {
        const std::string& key = it.first.toStdString();
        const std::string& value = it.second.convert<std::string>();
        json_msg[key] = value;
    }

    Json::StreamWriterBuilder wbuilder;
    std::string json_str = Json::writeString(wbuilder, json_msg);
    return json_str;
}


NtSubscriberBridge::NtSubscriberBridge(ros::NodeHandle& nh, std::shared_ptr<NetworkTable> nt_table, const std::vector<std::string>& topics, unsigned int queue_size)
      : nh_(nh), nt_table_(nt_table), queue_size_(queue_size) {
    for (const auto& topic : topics) {
        ros::Subscriber sub = nh_.subscribe<topic_tools::ShapeShifter>(topic, queue_size_, boost::bind(&NtSubscriberBridge::process_message, this, topic, _1));
        subscribers_.push_back(sub);
    }
}

void NtSubscriberBridge::process_message(const std::string& topic, const topic_tools::ShapeShifter::ConstPtr& msg) {
    // you must register the topic definition.
    // Don't worry, it will not be done twice
    parsers.registerParser(topic, msg);

    ParsersCollection::DeserializedMsg* deserialized_msg = parsers.deserialize(topic, msg);

    topic_tools::ShapeShifter::Ptr message = msg->instantiate(msg->datatype());
    if (message) {
        std::string json_str = rosMessageToJson(deserialized_msg, parser_);
        std::string base64_str = base64_encode(reinterpret_cast<const unsigned char*>(json_str.c_str()), json_str.length());
        nt_table_->PutString("temp", base64_str);
    } else {
        ROS_WARN("Failed to instantiate message for type: %s", msg->datatype().c_str());
    }
}
