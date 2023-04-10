#include "nt_publisher_bridge.h"


bool jsonToRosMessage(const std::string& json_str, topic_tools::ShapeShifter& message, const RosIntrospection::Parser& parser) {
    Json::CharReaderBuilder rbuilder;
    std::string errs;
    Json::Value json_msg;
    std::istringstream iss(json_str);
    if (!Json::parseFromStream(rbuilder, iss, &json_msg, &errs)) {
        ROS_ERROR("Failed to parse JSON: %s", errs.c_str());
        return false;
    }

    return true;
}

NtPublisherBridge::NtPublisherBridge(ros::NodeHandle& nh, std::shared_ptr<NetworkTable> nt_table, const std::vector<std::string>& nt_keys, unsigned int queue_size)
      : nh_(nh), nt_table_(nt_table), queue_size_(queue_size) {
    nt_table->AddEntryListener(
    "",
    [this](const nt::EntryNotification& event) {
        if (std::find(nt_keys.begin(), nt_keys.end(), event.name) != nt_keys.end()) {
        process_nt_entry(event.name, event.value->GetString());
        }
    },
    NT_NOTIFY_NEW | NT_NOTIFY_UPDATE);
}

void NtPublisherBridge::process_nt_entry(const std::string& nt_key, const std::string& base64_msg) {
    std::string json_str = base64_decode(value);
    
    if (message) {
        if (jsonToRosMessage(json_str, )) {
            // Publish ROS message
            if (publishers_.count(nt_key) == 0) {
                ros::Publisher pub = nh_.advertise<topic_tools::ShapeShifter>(nt_key, queue_size_);
                publishers_[nt_key] = pub;
            }
            publishers_[nt_key].publish(__);
        } else {
            ROS_WARN("Failed to deserialize JSON into ROS message");
        }
        delete message;
    } else {
        ROS_WARN("Failed to instantiate message for type: %s", );
    }
}
