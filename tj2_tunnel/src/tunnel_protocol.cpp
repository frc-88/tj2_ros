#include "tunnel_protocol.h"


TunnelProtocol::TunnelProtocol(std::map<string, string> categories)
{
    _read_packet_num = -1;
    _read_buffer_index = 0;
    _categories = categories;
}

TunnelProtocol::~TunnelProtocol()
{

}

int TunnelProtocol::parseBuffer(char* buffer, int start_index, int stop_index)
{
    int last_packet_index = 0;
    int index;
    for (index = start_index; index < stop_index; index++) {
        int packet_start = index;
        if (buffer[index] != PACKET_START_0) {
            ROS_DEBUG("Index %d is not PACKET_START_0", index);
            continue;
        }
        index++;
        if (index >= stop_index) {
            index = stop_index;
            continue;
        }
        if (buffer[index] != PACKET_START_1) {
            ROS_DEBUG("Index %d is not PACKET_START_1", index);
            continue;
        }
        index++;
        if (index >= stop_index) {
            index = stop_index;
            continue;
        }

        uint16_union_t union_packet_len;
        union_packet_len.byte[1] = buffer[index++];
        union_packet_len.byte[0] = buffer[index++];
        uint16_t length = union_packet_len.integer;

        if (index >= stop_index) {
            index = stop_index;
            ROS_DEBUG("Buffer length exceeded while searching for length start");
            continue;
        }
        
        ROS_DEBUG("Found packet length: %d", length);
        index += length;

        if (index > stop_index) {
            index = stop_index;
            ROS_DEBUG("Buffer length exceeded while searching for length stop. %d > %d", index, stop_index);
            continue;
        }
        if (buffer[index] != PACKET_STOP) {
            ROS_DEBUG("Buffer does not end with PACKET_STOP");
            continue;
        }
        index++;
        last_packet_index = index;
        ROS_DEBUG("Found a packet: %s", packetToString(buffer, packet_start, index).c_str());
        PacketResult* result = parsePacket(buffer, packet_start, index);
        _result_queue.push_back(result);
    }

    return last_packet_index;
}

PacketResult* TunnelProtocol::popResult()
{
    PacketResult* result = _result_queue.at(0);
    _result_queue.erase(_result_queue.begin());
    return result;
}

bool TunnelProtocol::isCodeError(int error_code)
{
    switch (error_code) {
        case NO_ERROR:
        case PACKET_COUNT_NOT_SYNCED_ERROR:
        case NULL_ERROR:
            return false;
        default:
            return true;
    }
}

string TunnelProtocol::packetToString(char* buffer, int start_index, int stop_index)
{
    string str = "";
    for (size_t i = start_index; i < stop_index; i++) {
        str += format_char(buffer[i]);
    }
    return str;
}

PacketResult* TunnelProtocol::parsePacket(char* buffer, int start_index, int stop_index)
{
    _read_buffer_index = start_index;
    ros::Time recv_time = ros::Time::now();
    int length = stop_index - start_index;
    if (length < MIN_PACKET_LEN) {
        ROS_INFO("Packet is not the minimum length (%d): %s", MIN_PACKET_LEN, packetToString(buffer, start_index, stop_index).c_str());
        return new PacketResult(PACKET_TOO_SHORT_ERROR, recv_time);
    }

    if (buffer[0] != PACKET_START_0) {
        ROS_INFO("Packet does not start with PACKET_START_0: %s", packetToString(buffer, start_index, stop_index).c_str());
        _read_packet_num++;
        return new PacketResult(PACKET_0_ERROR, recv_time);
    }
    _read_buffer_index++;
    if (buffer[1] != PACKET_START_1) {
        ROS_INFO("Packet does not start with PACKET_START_1: %s", packetToString(buffer, start_index, stop_index).c_str());
        _read_packet_num++;
        return new PacketResult(PACKET_1_ERROR, recv_time);
    }
    _read_buffer_index++;
    if (buffer[stop_index - 1] != PACKET_STOP) {
        ROS_INFO("Packet does not start with PACKET_STOP: %s", packetToString(buffer, start_index, stop_index).c_str());
        _read_packet_num++;
        return new PacketResult(PACKET_STOP_ERROR, recv_time);
    }

    _read_buffer_index += LENGTH_BYTE_LENGTH;
    uint8_t calc_checksum = 0;
    // compute checksum using all characters except the checksum itself
    for (int index = _read_buffer_index; index < stop_index - 3; index++) {
        calc_checksum += (uint8_t)buffer[index];
    }

    uint8_t recv_checksum = from_checksum(buffer + stop_index - 3);

    if (calc_checksum != recv_checksum) {
        ROS_INFO("Checksum failed! recv %02x != calc %02x. %s", recv_checksum, calc_checksum, packetToString(buffer, start_index, stop_index).c_str());
        _read_packet_num++;
        return new PacketResult(CHECKSUMS_DONT_MATCH_ERROR, recv_time);
    }

    if (!getNextSegment(buffer, stop_index, 4)) {
        ROS_WARN("Failed to find packet number segment! %s", packetToString(buffer, start_index, stop_index).c_str());
        _read_packet_num++;
        return new PacketResult(PACKET_COUNT_NOT_FOUND_ERROR, recv_time);
    }

    uint32_t recv_packet_num = to_uint32(buffer + _current_segment_start);

    if (_read_packet_num == -1) {
        _read_packet_num = recv_packet_num;
    }

    PacketResult* result = new PacketResult(NO_ERROR, recv_time);

    if (recv_packet_num != _read_packet_num) {
        ROS_WARN("Received packet num doesn't match local count. recv %d != local %d", recv_packet_num, _read_packet_num);
        _read_packet_num++;
        _read_packet_num = recv_packet_num;
        result->setErrorCode(PACKET_COUNT_NOT_SYNCED_ERROR);
    }

    if (!getNextSegment(buffer, stop_index)) {
        ROS_WARN(
            "Failed to find category segment %s! %s",
            packetToString(buffer, _current_segment_start, _current_segment_stop).c_str(),
            packetToString(buffer, start_index, stop_index).c_str()
        );
        _read_packet_num++;
        return new PacketResult(PACKET_CATEGORY_ERROR, recv_time);
    }

    string category = to_string(buffer + _current_segment_start, _current_segment_stop - _current_segment_start);
    if (category.size() == 0) {
        ROS_WARN(
            "Category segment is empty: %s, %s",
            packetToString(buffer, _current_segment_start, _current_segment_stop).c_str(),
            packetToString(buffer, start_index, stop_index).c_str()
        );
        _read_packet_num++;
        return new PacketResult(PACKET_CATEGORY_ERROR, recv_time);
    }
    result->setCategory(category);

    std::map<std::string, string>::iterator it = _categories.find(category);
    if (it == _categories.end()) {
        ROS_WARN(
            "'%s' is not a recognized category: %s",
            category.c_str(),
            packetToString(buffer, start_index, stop_index).c_str()
        );
        _read_packet_num++;
        return new PacketResult(PACKET_CATEGORY_ERROR, recv_time);
    }

    string packet_format = it->second;

    for (int format_index = 0; format_index < packet_format.size(); format_index++)
    {
        if (!parseNextSegment(buffer, stop_index, packet_format.at(format_index), result)) {
            ROS_WARN(
                "Failed to parse segment #%d. Buffer: %s",
                format_index,
                packetToString(buffer, start_index, stop_index).c_str()
            );
            _read_packet_num++;
            return new PacketResult(INVALID_FORMAT_ERROR, recv_time);
        }
    }
    result->setErrorCode(NO_ERROR);
    _read_packet_num++;
    
    return result;
}

bool TunnelProtocol::getNextSegment(char* buffer, int stop_index, int length)
{
    if (_read_buffer_index >= stop_index + length) {
        return false;
    }
    if (length == -1) {
        length = to_uint16(buffer + _read_buffer_index);
        _read_buffer_index += 2;
        if (length >= stop_index + length) {
            ROS_ERROR("Parsed length %d exceeds buffer length! %d", length, _read_buffer_index + length);
            return false;
        }
    }
    _current_segment_start = _read_buffer_index;
    _read_buffer_index += length;
    _current_segment_stop = _read_buffer_index;
    return true;
}

bool TunnelProtocol::getNextSegment(char* buffer, int stop_index)
{
    if (_read_buffer_index >= stop_index) {
        return false;
    }
    int sep_index;
    for (sep_index = _read_buffer_index; sep_index < stop_index; sep_index++) {
        if (buffer[sep_index] == PACKET_SEP) {
            break;
        }
    }
    if (sep_index >= stop_index) {
        _current_segment_start = _read_buffer_index;
        _current_segment_stop = stop_index;
        _read_buffer_index = stop_index;
    }
    else {
        _current_segment_start = _read_buffer_index;
        _current_segment_stop = sep_index;
        _read_buffer_index = sep_index + 1;
    }
    return true;
}


bool TunnelProtocol::parseNextSegment(char* buffer, int stop_index, char format, PacketResult* result) {
    int length;
    if (format == 'd' || format == 'u') {
        length = 4;
    }
    else if (format == 'f') {
        length = 8;
    }
    else {
        length = -1;
    }
    if (!getNextSegment(buffer, stop_index, length)) {
        return false;
    }

    result->add(buffer + _current_segment_start);

    if (format != 'd' && format != 'u' && format != 's' && format != 'x' && format != 'f') {
        ROS_ERROR("Failed to parse segment '%s' as '%c'", packetToString(buffer, _current_segment_start, _current_segment_stop).c_str(), format);
    }
    return true;
}

