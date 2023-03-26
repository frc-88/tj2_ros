
#include "tj2_networktables.h"

TJ2NetworkTables::TJ2NetworkTables(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<string>("~nt_server", _nt_server, "");
    ros::param::param<int>("~nt_port", _nt_port, 5800);
    ros::param::param<double>("~update_interval", _update_interval, 0.01);

    _send_topics = get_topics("send_topics");
    _recv_topics = get_topic_map("recv_topics");

    setup_nt_connection(_nt_server, _nt_port);
    subscribe_to_send_topics(_send_topics);
    advertise_on_recv_topics(_recv_topics);

    ROS_INFO("tj2_networktables is ready");
}

// ---
// Sub callbacks
// ---
void TJ2NetworkTables::send_topic_callback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string &topic_name)
{
    // https://github.com/facontidavide/type_introspection_tests/blob/master/example/multi_subscriber.cpp
    static std::vector<uint8_t> buffer;
    buffer.resize(msg->size());
    ros::serialization::OStream stream(buffer.data(), buffer.size());
    msg->write(stream);
    const char* data = (char*)stream.getData();
    nt::SetEntryValue(_nt_publishers[topic_name], nt::Value::MakeRaw(llvm::StringRef(data, stream.getLength())));
}

void TJ2NetworkTables::recv_topic_callback(const nt::EntryNotification& notification)
{
    static uint8_t* buffer;
    boost::shared_array<uint8_t> array_buffer(buffer);
    string topic_name = notification.name;
    size_t length = get_entry_raw(notification.entry, &buffer);
    if (!_publishers.count(topic_name)) {
        _publishers[topic_name] = shape_shifter.advertise(nh, topic_name, _recv_topics[topic_name].queue_size);
    }
    ros::SerializedMessage msg(array_buffer, length);
    ros::TopicManager::instance()->publish(topic_name, boost::bind(serializeMessage<ros::SerializedMessage>, boost::ref(msg)), msg);

}

// ---
// Helpers
// ---

void TJ2NetworkTables::setup_nt_connection(string server_name, unsigned int port)
{
    bool is_server = (
        (server_name.compare("") == 0) ||
        (server_name.compare("0.0.0.0") == 0) ||
        (server_name.compare("127.0.0.1") == 0)
    );
    _nt = nt::GetDefaultInstance();
    nt::AddLogger(_nt,
                [](const nt::LogMessage& msg) {
                    // std::fputs(msg.message.c_str(), stderr);
                    // std::fputc('\n', stderr);
                    ROS_DEBUG("[NT]:\t %s", msg.message.c_str());
                },
                0, UINT_MAX
    );
    if (is_server) {
        nt::StartServer(_nt, "tj2_networktables.ini", "", _nt_port);
        ROS_INFO("Starting NT server on port %d", _nt_port);
    }
    else {
        nt::StartClient(_nt, server_name.c_str(), port);
        ROS_INFO("Connecting to NT server %s on port %d", server_name.c_str(), _nt_port);
    }
    nt::SetUpdateRate(_nt, _update_interval);
}

void TJ2NetworkTables::subscribe_to_send_topics(set<string> topic_names)
{
    for (const std::string& topic_name : topic_names)
    {
        ROS_INFO("Subscribing to %s", topic_name.c_str());
        boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
        callback = [this, topic_name](const topic_tools::ShapeShifter::ConstPtr& msg) -> void {
            this->send_topic_callback(msg, topic_name);
        };
        _subscribers[topic_name] = nh.subscribe(topic_name, 10, callback);
        _nt_publishers[topic_name] = nt::GetEntry(_nt, topic_name);
    }
}

void TJ2NetworkTables::advertise_on_recv_topics(map<string, TopicInfo_t> topic_info)
{
    for (auto iter = topic_info.begin(); iter != topic_info.end(); iter++) {
        string topic_name = iter->first;
        ROS_INFO("Advertising on %s", topic_name.c_str());
        TopicInfo_t info = iter->second;
        
        NT_Entry entry = nt::GetEntry(_nt, topic_name);
        _nt_subscribers[topic_name] = entry;
        nt::AddEntryListener(entry, 
            boost::bind(&TJ2NetworkTables::recv_topic_callback, this, _1),
            nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate
        );
    }
}

set<string> TJ2NetworkTables::get_topics(string param_name)
{
    string key;
    vector<string> value;
    if (!ros::param::search(param_name, key)) {
        ROS_ERROR("Failed to find %s parameter", param_name.c_str());
        std::exit(EXIT_FAILURE);
    }
    ROS_DEBUG("Found send_topics: %s", key.c_str());
    nh.getParam(key, value);

    set<string> result;
    for (size_t index = 0; index < value.size(); index++) {
        result.insert(value[index]);
    }
    return result;
}

map<string, TopicInfo_t> TJ2NetworkTables::get_topic_map(string param_name)
{
    XmlRpc::XmlRpcValue recv_topics_param;
    string key;
    if (!ros::param::search("recv_topics", key)) {
        throw std::runtime_error("Failed to find recv_topics parameter");
    }
    nh.getParam(key, recv_topics_param);

    if (recv_topics_param.getType() != XmlRpc::XmlRpcValue::Type::TypeArray ||
        recv_topics_param.size() == 0) {
        throw std::runtime_error("recv_topics is the wrong type or size");
    }

    map<string, TopicInfo_t> recv_topics;
    for (int index = 0; index < recv_topics_param.size(); index++) {
        if (recv_topics_param[index].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("recv_topics element is not a struct");
        }
        TopicInfo_t info;
        info.topic_name = (string)recv_topics_param[index]["topic_name"];
        info.queue_size = (int)recv_topics_param[index]["queue_size"];
        recv_topics[info.topic_name] = info;
    }
    return recv_topics;
}

size_t TJ2NetworkTables::get_entry_raw(NT_Entry entry, uint8_t** buffer)
{
    auto value = nt::GetEntryValue(entry);
    if (value == nullptr) {
        ROS_WARN_THROTTLE(1.0, "NT entry is NULL. Expected a string!");
        return 0;
    }
    else if (!value->IsRaw()) {
        ROS_WARN_THROTTLE(1.0, "NT entry is not bytes as expected! Got type %d", value->type());
        return 0;
    }
    else {
        llvm::StringRef ref = value->GetRaw();
        *buffer = (uint8_t*)ref.data();
        return ref.size();
    }
}

int TJ2NetworkTables::run()
{
    ros::spin();
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_networktables");
    ros::NodeHandle nh;
    TJ2NetworkTables node(&nh);
    return node.run();
}
