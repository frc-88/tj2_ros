
#include "tj2_networktables.h"

TJ2NetworkTables::TJ2NetworkTables(ros::NodeHandle* nodehandle) :
    nh(*nodehandle), _tf_listener(_tf_buffer)
{
    ros::param::param<int>("~nt_port", _nt_port, 5800);
    ros::param::param<double>("~update_interval", _update_interval, 0.01);

    _send_topics = get_topics("send_topics");
    _recv_topics = get_topics("recv_topics");

    setup_nt_server();
    subscribe_to_send_topics(_send_topics);
    advertise_on_recv_topics(_recv_topics);

    ROS_INFO("tj2_networktables is ready");
}

// ---
// Sub callbacks
// ---
void TJ2NetworkTables::send_topic_callback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string &topic_name)
{
    std::stringstream ss;
    msg->read(ss);
    string send_string = ss.str();
    nt::SetEntryValue(_nt_publishers[topic_name], nt::Value::MakeString(send_string));
}

void TJ2NetworkTables::recv_topic_callback(const nt::EntryNotification& notification)
{

}

// ---
// Helpers
// ---

void TJ2NetworkTables::setup_nt_server()
{
    _nt = nt::GetDefaultInstance();
    nt::AddLogger(_nt,
                [](const nt::LogMessage& msg) {
                    // std::fputs(msg.message.c_str(), stderr);
                    // std::fputc('\n', stderr);
                    ROS_DEBUG("[NT]:\t %s", msg.message.c_str());
                },
                0, UINT_MAX
    );
    nt::StartServer(_nt, "tj2_networktables.ini", "", _nt_port);
    nt::SetUpdateRate(_nt, _update_interval);

}

void TJ2NetworkTables::subscribe_to_send_topics(set<string> topic_names)
{
    for (const std::string& topic_name : topic_names)
    {
        boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
        callback = [topic_name](const topic_tools::ShapeShifter::ConstPtr& msg) -> void {
            this.send_topic_callback(msg, topic_name);
        };
        subscribers[topic_name] = nh.subscribe(topic_name, 10, callback);
        _nt_publishers[topic_name] = nt::GetEntry(_nt, topic_name);
    }
}

void TJ2NetworkTables::advertise_on_recv_topics(set<string> topic_names)
{
    for (const std::string& topic_name : topic_names)
    {
        entry = nt::GetEntry(_nt, topic_name);
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


int TJ2NetworkTables::run()
{
    ros::Rate clock_rate(_publish_rate);  // Hz
    while (ros::ok())
    {
        clock_rate.sleep();
        
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_networktables");
    ros::NodeHandle nh;
    TJ2NetworkTables node(&nh);
    return node.run();
}
