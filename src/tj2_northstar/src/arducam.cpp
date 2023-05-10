#include <arducam.h>

Arducam::Arducam(int device_num, std::string fourcc_code, int width, int height, int channel)
{
    this->device_num = device_num;
    this->width = width;
    this->height = height;
    this->channel = channel;
    this->is_open = false;
    this->depth = 0;
    this->convert_to_rgb = false;
    this->convert_code = -1;
    this->fourcc_code = fourcc_code;
}

Arducam::~Arducam()
{
    stop();
}


bool does_file_exist(const std::string& path) {
    struct stat buffer;   
    return (stat(path.c_str(), &buffer) == 0); 
}

std::string read_file_to_string(const std::string& path) {
    std::ifstream filestream(path);
    std::stringstream buffer;
    buffer << filestream.rdbuf();
    return buffer.str();
}

int to_fourcc(char a, char b, char c, char d) {
    return a | (b << 8) | (c << 16) | (d << 24);
}

int to_format_value(std::string code)
{
    while (code.size() < 4) {
        code += " ";
    }
    if (code.size() > 4) {
        std::cerr << "Invalid format code: " << code << std::endl;
        throw std::runtime_error("Invalid format code");
    }
    return to_fourcc(code.at(0), code.at(1), code.at(2), code.at(3));
}

int open_video_device(int device_num) {
    std::string device_path = "/dev/video" + std::to_string(device_num);
    int vd = open(device_path.c_str(), O_RDWR);
    if (vd < 0) {
        std::cerr << "Error opening video device: " << device_path << std::endl;
        throw std::runtime_error("Error opening video device");
    }
    return vd;
}

int read_dev(int vd, uint16_t reg, uint32_t &val) {
    arducam_dev dev;
    dev.reg = reg;
    int ret = ioctl(vd, VIDIOC_R_DEV, &dev);
    val = dev.val;
    return ret;
}

int write_dev(int vd, uint16_t reg, uint32_t val) {
    arducam_dev dev;
    dev.reg = reg;
    dev.val = val;
    return ioctl(vd, VIDIOC_W_DEV, &dev);
}

int read_sensor(int vd, uint16_t reg, uint16_t &val) {
    arducam_i2c i2c;
    i2c.reg = reg;
    int ret = ioctl(vd, VIDIOC_R_I2C, &i2c);
    val = i2c.val;
    return ret;
}

int write_sensor(int vd, uint16_t reg, uint16_t val) {
    arducam_i2c i2c;
    i2c.reg = reg;
    i2c.val = val;
    return ioctl(vd, VIDIOC_W_I2C, &i2c);
}

int get_pixel_format(int vd, uint32_t &pixel_format) {
    v4l2_format fmt;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int ret = ioctl(vd, VIDIOC_G_FMT, &fmt);
    pixel_format = fmt.fmt.pix.pixelformat;
    return ret;
}

bool Arducam::start() {
    if (is_open) {
        std::cout << "Arducam is already started" << std::endl;
        return false;
    }
    std::cout << "Opening video device " << this->device_num << std::endl;
    this->capture = cv::VideoCapture(this->device_num, cv::CAP_V4L2);
    if (!this->capture.isOpened()){
        std::cerr << "Error opening video capture: " << this->device_num << std::endl;
        throw std::runtime_error("Error opening video capture");
        return -1;
    }
    this->video_device = open_video_device(this->device_num);
    this->capture.set(cv::CAP_PROP_FOURCC, to_format_value(fourcc_code));

    update_pixel_format();

    if (this->convert_to_rgb == 0) {
        this->capture.set(cv::CAP_PROP_CONVERT_RGB, 0);
    }
    if (width > 0) {
        this->capture.set(cv::CAP_PROP_FRAME_WIDTH, width);
    }
    if (height > 0) {
        this->capture.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    }

    if (0 <= channel && channel < 4) {
        write_dev(this->video_device, CHANNEL_SWITCH_REG, channel);
    }

    is_open = true;
    std::cout << "Opened device " << this->device_num << " successfully" << std::endl;
    return true;
}

void Arducam::set_parameter(std::string name, int value) {
    std::stringstream command;
    command << "/usr/bin/v4l2-ctl -d /dev/video" << this->device_num << " -c " << name << "=" << value;
    std::system(command.str().c_str());
}

void Arducam::set_frame_rate(int frame_rate) {
    set_parameter("frame_rate", frame_rate);
}

void Arducam::set_frame_timeout(int frame_timeout) {
    set_parameter("frame_timeout", frame_timeout);
}

void Arducam::set_low_latency_mode(bool low_latency_mode) {
    set_parameter("low_latency_mode", (int)low_latency_mode);
}

void Arducam::set_exposure(int exposure) {
    set_parameter("exposure", exposure);
}

void Arducam::set_analogue_gain(int analogue_gain) {
    set_parameter("analogue_gain", analogue_gain);
}

bool Arducam::stop() {
    if (!is_open) {
        std::cerr << "Arducam is already stopped" << std::endl;
        return false;
    }
    std::cout << "Closing video device " << this->device_num << std::endl;
    this->capture.release();
    if (this->video_device >= 0) {
        close(this->video_device);
    }
    is_open = false;
    return true;
}

bool Arducam::read(cv::Mat &result_frame)
{
    cv::Mat frame;
    bool success = this->capture.read(frame);
    
    if (!success) {
        return false;
    }

    cv::cvtColor(convert(frame), result_frame, cv::COLOR_BGR2GRAY);
    return true;

}

cv::Mat Arducam::convert(const cv::Mat &frame) {
    if (convert_to_rgb) {
        return frame;
    }

    cv::Mat converted_frame = frame.clone();

    if (depth != -1) {
        double scale = 256.0 / static_cast<double>(1 << depth);
        converted_frame.convertTo(converted_frame, CV_8U, scale);
    }

    if (convert_code != -1) {
        cv::cvtColor(converted_frame, converted_frame, convert_code);
    }

    return converted_frame;
}

void Arducam::update_pixel_format()
{
    uint32_t pixel_format;
    int result = get_pixel_format(this->video_device, pixel_format);
    int code = get_convert_code(pixel_format);

    if (code != -1) {
        convert_code = code;
        depth = -1;
    }
    else {
        v4l2_fmtdesc fmtdesc;
        fmtdesc.index = 0;
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        while (ioctl(this->video_device, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
            convert_code = get_convert_code(fmtdesc.pixelformat);
            fmtdesc.index++;
        }
    }

    if (convert_code == -1) {
        convert_to_rgb = true;
        depth = -1;
    }
    else {
        convert_to_rgb = false;
        if (is_nvidia_nx()) {
            depth = 16;
        }
        else {
            depth = 10;
        }
    }
}

int get_convert_code(uint32_t pixel_format) {
    switch (pixel_format)
    {
    case V4L2_PIX_FMT_SBGGR8:
    case V4L2_PIX_FMT_SBGGR10:
        return cv::COLOR_BayerRG2BGR;
    case V4L2_PIX_FMT_SGBRG8:
    case V4L2_PIX_FMT_SGBRG10:
        return cv::COLOR_BayerGR2BGR;
    case V4L2_PIX_FMT_SGRBG8:
    case V4L2_PIX_FMT_SGRBG10:
        return cv::COLOR_BayerGB2BGR;
    case V4L2_PIX_FMT_SRGGB8:
    case V4L2_PIX_FMT_SRGGB10:
        return cv::COLOR_BayerBG2BGR;

    default:
        return -1;
    }
}

bool Arducam::is_nvidia_nx()
{
    std::string path = "/proc/device-tree/model";
    if (!does_file_exist(path)) {
        return false;
    }
    std::string contents = read_file_to_string(path);
    return contents.find("NVIDIA") != std::string::npos && contents.find("NX") != std::string::npos;
}
