#include "tj2_tunnel/tj2_tunnel.h"

TJ2Tunnel::TJ2Tunnel(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<string>("~host", _host, "127.0.0.1");
    ros::param::param<int>("~port", _port, 3000);

    ros::param::param<double>("~remote_linear_units_conversion", _remote_linear_units_conversion, 0.3048);
    ros::param::param<double>("~remote_angular_units_conversion", _remote_angular_units_conversion, M_PI / 180.0);

    // _categories_param is a map
    if (_categories_param.getType() != XmlRpc::XmlRpcValue::Type::TypeStruct ||
        _categories_param.size() == 0) {
        ROS_ERROR("categories wrong type or size");
        return;
    }
    
    for (XmlRpc::XmlRpcValue::iterator it = _categories_param.begin(); it != _categories_param.end(); ++it)
    {
        string category = it->first;
        string format = it->second;
        _categories[category] = format;
    }

    _read_buffer = new char[READ_BUFFER_LEN];
    _socket_initialized = false;
    _socket_id = 0;

    protocol = new TunnelProtocol(_categories);

    if ((_socket_id = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        ROS_ERROR("Socket creation error! Failed to create connection.");
        return;
    }

    _serv_addr.sin_family = AF_INET;
    _serv_addr.sin_port = htons(_port);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if(inet_pton(AF_INET, _host.c_str(), &_serv_addr.sin_addr) <= 0)
    {
        ROS_ERROR("Socket creation error. Invalid server address.");
        return;
    }

    if (connect(_socket_id, (struct sockaddr *)&_serv_addr, sizeof(_serv_addr)) < 0)
    {
        ROS_ERROR("Socket connection failed!");
        return;
    }

    _unparsed_index = 0;

    _socket_initialized = true;
    ROS_INFO("tj2_networktables init complete");
}

void TJ2Tunnel::packetCallback(PacketResult* result)
{
    string category = result->getCategory();
    if (category.compare("odom")) {
        
    }
    else if (category.compare("ping")) {
        
    }
}


bool TJ2Tunnel::loop()
{
    if (!_socket_initialized) {
        ROS_WARN("Socket is not initialized. Exiting.");
        return false;
    }
    int num_chars_read = read(_socket_id, _read_buffer, READ_BUFFER_LEN);
    if (num_chars_read == -1) {
        ROS_WARN("Socket indicated it should exit.");
        return false;
    }
    int last_parsed_index = protocol->parseBuffer(_read_buffer, 0, _unparsed_index + num_chars_read);
    if (last_parsed_index > 0) {
        for (int index = last_parsed_index, shifted_index = 0; index < READ_BUFFER_LEN; index++, shifted_index++) {
            _read_buffer[shifted_index] = _read_buffer[index];
        }
    }
    _unparsed_index = _unparsed_index + num_chars_read - last_parsed_index;
    if (_unparsed_index >= READ_BUFFER_LEN) {
        _unparsed_index = 0;
    }

    PacketResult* result;
    do {
        result = protocol->popResult();
        if (result->getErrorCode() == TunnelProtocol::NULL_ERROR) {
            continue;
        }
        if (protocol->isCodeError(result->getErrorCode())) {
            ROS_ERROR("Encountered error code %d.", result->getErrorCode());
            continue;
        }
        packetCallback(result);
    }
    while (result->getErrorCode() != -1);
    return true;
}

int TJ2Tunnel::run()
{
    ros::Rate clock_rate(100);  // Hz

    int exit_code = 0;
    while (ros::ok())
    {
        // let ROS process any events
        ros::spinOnce();
        clock_rate.sleep();

        try {
            if (!loop()) {
                break;
            }
        }
        catch (exception& e) {
            ROS_ERROR_STREAM("Exception in main loop: " << e.what());
            exit_code = 1;
            break;
        }
    }
    return exit_code;
}
