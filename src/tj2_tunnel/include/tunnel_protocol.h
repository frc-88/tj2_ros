#pragma once

#include <limits.h>
#include "ros/ros.h"

using namespace std;

#define THROW_EXCEPTION(msg)  throw std::runtime_error(msg)


typedef union uint16_union
{
    uint16_t integer;
    unsigned char byte[sizeof(uint16_t)];
} uint16_union_t;

typedef union uint32_union
{
    uint32_t integer;
    unsigned char byte[sizeof(uint32_t)];
} uint32_union_t;


typedef union int32_union
{
    int32_t integer;
    unsigned char byte[sizeof(int32_t)];
} int32_union_t;

typedef union double_union
{
    double floating_point;
    unsigned char byte[sizeof(double)];
} double_union_t;


uint32_t to_uint32(char* buffer)
{
    uint32_union_t union_data;
    for (unsigned short i = 0; i < sizeof(uint32_t); i++) {
        union_data.byte[sizeof(uint32_t) - i - 1] = buffer[i];
    }
    return union_data.integer;
}

uint16_t to_uint16(char* buffer)
{
    uint32_union_t union_data;
    for (unsigned short i = 0; i < sizeof(uint16_t); i++) {
        union_data.byte[sizeof(uint16_t) - i - 1] = buffer[i];
    }
    return union_data.integer;
}

int16_t to_int32(char* buffer)
{
    int32_union_t union_data;
    for (unsigned short i = 0; i < sizeof(int32_t); i++) {
        union_data.byte[sizeof(int32_t) - i - 1] = buffer[i];
    }
    return union_data.integer;
}

double to_double(char* buffer)
{
    double_union_t union_data;
    for (unsigned short i = 0; i < sizeof(double); i++) {
        union_data.byte[i] = buffer[i];
    }
    return union_data.floating_point;
}

string to_string(char* buffer, int length)
{
    if (length < 0) {
        ROS_ERROR("Can't convert string if length is less than zero");
        return "";
    }
    char char_array[length + 1];
    memcpy(char_array, buffer, length);
    char_array[length] = '\0';
    return string(char_array);
}

uint8_t from_checksum(char* buffer)
{
    char recv_checksum_array[3];
    memcpy(recv_checksum_array, buffer, 2);
    recv_checksum_array[2] = '\0';
    return strtol(recv_checksum_array, NULL, 16);
}


string format_char(unsigned char c)
{
    if (c == 92) return "\\\\";
    else if (c == 9) return "\\t";
    else if (c == 10) return "\\n";
    else if (c == 13) return "\\r";
    else if (c == 11 || c == 12 || c <= 9 || (14 <= c && c <= 31) || 127 <= c)
    {
        char* temp_buf = new char[4];
        sprintf(temp_buf, "\\x%02x", c);
        return string(temp_buf);
    }
    else {
        return string(1, (char)c);
    }
}

string packetToString(char* buffer, int start_index, int stop_index)
{
    string str = "";
    for (size_t i = start_index; i < stop_index; i++) {
        str += format_char(buffer[i]);
    }
    return str;
}

class PacketResult
{
private:
    string _category;
    int _error_code;
    ros::Time _recv_time;
    int _start_index;
    int _stop_index;
    char* _buffer;
    int _current_index;
    
    void checkIndex() {
        if (_current_index >= _stop_index) {
            ROS_ERROR("Index exceeds buffer limits. %d >= %d", _current_index, _stop_index);
            THROW_EXCEPTION("Index exceeds buffer limits");
        }
    }
public:
    PacketResult(int error_code, ros::Time recv_time) {
        _category = "";
        _recv_time = recv_time;
        _error_code = error_code;
    }

    ~PacketResult() {
        
    }

    void setCategory(string category) {
        _category = category;
    }
    string getCategory() {
        return _category;
    }
    void setErrorCode(int error_code) {
        if (_error_code != 0) {
            return;
        }
        _error_code = error_code;
    }
    int getErrorCode() {
        return _error_code;
    }
    void setRecvTime(ros::Time recv_time) {
        _recv_time = recv_time;
    }
    ros::Time getRecvTime() {
        return _recv_time;
    }
    void setBuffer(char* buffer) {
        _buffer = buffer;
    }
    void setStart(int index) {
        _start_index = index;
        _current_index = _start_index;
    }
    void setStop(int index) {
        _stop_index = index;
    }
    int32_t getInt() {
        int32_t result = to_int32(_buffer + _current_index);
        _current_index += sizeof(int32_t);
        checkIndex();
        return result;
    }
    double getDouble() {
        double result = to_double(_buffer + _current_index);
        _current_index += sizeof(double);
        checkIndex();
        return result;
    }
    string getString() {
        int length = to_uint16(_buffer + _current_index);
        _current_index += sizeof(uint16_t);
        checkIndex();
        return getString(length);
    }
    string getString(int length) {
        string result = to_string(_buffer + _current_index, length);
        _current_index += length;
        checkIndex();
        return result;
    }
};


class TunnelProtocol
{
private:
    vector<PacketResult*> _result_queue;
    int _read_buffer_index;
    uint32_t _read_packet_num;
    uint32_t _write_packet_num;
    int _current_segment_start;
    int _current_segment_stop;

    PacketResult* parsePacket(char* buffer, int start_index, int stop_index);
    bool parseNextSegment(char* buffer, int stop_index, char format, PacketResult* result);
    bool getNextSegment(char* buffer, int stop_index);
    bool getNextSegment(char* buffer, int stop_index, int length);

public:
    static const char PACKET_START_0 = 0x12;
    static const char PACKET_START_1 = 0x13;
    static const char PACKET_STOP = '\n';
    static const char PACKET_SEP = '\t';
    static const int MAX_PACKET_LEN = 1024;
    static const int MIN_PACKET_LEN = 13;
    static const int MAX_SEGMENT_LEN = 64;
    static const int CHECKSUM_START_INDEX = 4;
    static const int LENGTH_START_INDEX = 2;
    static const int LENGTH_BYTE_LENGTH = 2;

    static const int NULL_ERROR = -1;
    static const int NO_ERROR = 0;
    static const int PACKET_0_ERROR = 1;
    static const int PACKET_1_ERROR = 2;
    static const int PACKET_TOO_SHORT_ERROR = 3;
    static const int CHECKSUMS_DONT_MATCH_ERROR = 4;
    static const int PACKET_COUNT_NOT_FOUND_ERROR = 5;
    static const int PACKET_COUNT_NOT_SYNCED_ERROR = 6;
    static const int PACKET_CATEGORY_ERROR = 7;
    static const int INVALID_FORMAT_ERROR = 8;
    static const int PACKET_STOP_ERROR = 9;
    static const int SEGMENT_TOO_LONG_ERROR = 10;
    static const int PACKET_TIMEOUT_ERROR = 11;

    TunnelProtocol();
    ~TunnelProtocol();

    int parseBuffer(char* buffer, int start_index, int stop_index);
    PacketResult* popResult();
    bool isCodeError(int error_code);
    int makePacket(char* write_buffer, string category, const char *formats, va_list args);
};
