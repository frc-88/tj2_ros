#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string>
#include <iostream>
#include <fstream>
#include <linux/videodev2.h>

#include <sys/stat.h>
#include <opencv2/opencv.hpp>


const int IOC_NRBITS = 8;
const int IOC_TYPEBITS = 8;
const int IOC_SIZEBITS = 14;
const int IOC_DIRBITS = 2;

const int IOC_NRSHIFT = 0;
constexpr int IOC_TYPESHIFT = IOC_NRSHIFT + IOC_NRBITS;
constexpr int IOC_SIZESHIFT = IOC_TYPESHIFT + IOC_TYPEBITS;
constexpr int IOC_DIRSHIFT = IOC_SIZESHIFT + IOC_SIZEBITS;

const int IOC_NONE = 0;
const int IOC_WRITE = 1;
const int IOC_READ = 2;

const int DEVICE_REG_BASE = 0x0100;
const int PIXFORMAT_REG_BASE = 0x0200;
const int FORMAT_REG_BASE = 0x0300;
const int CTRL_REG_BASE = 0x0400;
const int SENSOR_REG_BASE = 0x500;

constexpr int STREAM_ON = DEVICE_REG_BASE | 0x0000;
constexpr int FIRMWARE_VERSION_REG = DEVICE_REG_BASE | 0x0001;
constexpr int SENSOR_ID_REG = DEVICE_REG_BASE | 0x0002;
constexpr int DEVICE_ID_REG = DEVICE_REG_BASE | 0x0003;
constexpr int FIRMWARE_SENSOR_ID_REG = DEVICE_REG_BASE | 0x0005;
constexpr int SERIAL_NUMBER_REG = DEVICE_REG_BASE | 0x0006;
constexpr int CHANNEL_SWITCH_REG = DEVICE_REG_BASE | 0x0008;

constexpr int PIXFORMAT_INDEX_REG = PIXFORMAT_REG_BASE | 0x0000;
constexpr int PIXFORMAT_TYPE_REG = PIXFORMAT_REG_BASE | 0x0001;
constexpr int PIXFORMAT_ORDER_REG = PIXFORMAT_REG_BASE | 0x0002;
constexpr int MIPI_LANES_REG = PIXFORMAT_REG_BASE | 0x0003;

constexpr int RESOLUTION_INDEX_REG = FORMAT_REG_BASE | 0x0000;
constexpr int FORMAT_WIDTH_REG = FORMAT_REG_BASE | 0x0001;
constexpr int FORMAT_HEIGHT_REG = FORMAT_REG_BASE | 0x0002;

constexpr int CTRL_INDEX_REG = CTRL_REG_BASE | 0x0000;
constexpr int CTRL_ID_REG = CTRL_REG_BASE | 0x0001;
constexpr int CTRL_MIN_REG = CTRL_REG_BASE | 0x0002;
constexpr int CTRL_MAX_REG = CTRL_REG_BASE | 0x0003;
constexpr int CTRL_STEP_REG = CTRL_REG_BASE | 0x0004;
constexpr int CTRL_DEF_REG = CTRL_REG_BASE | 0x0005;
constexpr int CTRL_VALUE_REG = CTRL_REG_BASE | 0x0006;

constexpr int SENSOR_RD_REG = SENSOR_REG_BASE | 0x0001;
constexpr int SENSOR_WR_REG = SENSOR_REG_BASE | 0x0002;

const int NO_DATA_AVAILABLE = 0xFFFFFFFE;

const int DEVICE_ID = 0x0030;

struct arducam_i2c {
    uint16_t reg;
    uint16_t val;
};

struct arducam_dev {
    uint16_t reg;
    uint32_t val;
};

#define VIDIOC_R_I2C _IOWR('V', BASE_VIDIOC_PRIVATE + 0, struct arducam_i2c)
#define VIDIOC_W_I2C _IOWR('V', BASE_VIDIOC_PRIVATE + 1, struct arducam_i2c)
#define VIDIOC_R_DEV _IOWR('V', BASE_VIDIOC_PRIVATE + 2, struct arducam_dev)
#define VIDIOC_W_DEV _IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct arducam_dev)

int open_video_device(int device_num);
int read_dev(int vd, uint16_t reg, uint32_t &val);
int write_dev(int vd, uint16_t reg, uint32_t val);
int read_sensor(int vd, uint16_t reg, uint16_t &val);
int write_sensor(int vd, uint16_t reg, uint16_t val);
int get_pixel_format(int vd, uint32_t &pixelformat);
int get_convert_code(uint32_t pixel_format);

bool does_file_exist(const std::string& path);
std::string read_file_to_string(const std::string& path);
int to_fourcc(char a, char b, char c, char d);
int to_format_value(std::string code);

class Arducam
{
private:
    int width, height, depth;
    int convert_code;
    bool convert_to_rgb;
    int channel;

    int device_num;
    bool is_open;
    std::string fourcc_code;

    cv::VideoCapture capture;
    int video_device;

    void update_pixel_format();
    bool is_nvidia_nx();
    cv::Mat convert(const cv::Mat &frame);
public:
    Arducam(int device_num, std::string fourcc_code, int width = -1, int height = -1, int channel = -1);
    ~Arducam();
    bool start();
    bool stop();
    bool read(cv::Mat &result_frame);
};
