#include <tj2_northstar.h>

SplitCam::SplitCam(
    std::string info_directory,
    std::string prefix,
    int index,
    ros::NodeHandle* nodehandle,
    image_transport::ImageTransport* transport)
{
    nh = nodehandle;
    _prefix = prefix;
    _index = index;
    _transport = transport;
    _pub = _transport->advertiseCamera(get_serial() + "/image_raw", 1);

    std::string path = info_directory + "/" + get_serial() + ".yaml";
    _info_did_load = load_camera_info(path);
}


template<size_t Size, class Container>
boost::array<typename Container::value_type, Size> as_array(const Container &cont)
{
    assert(cont.size() == Size);
    boost::array<typename Container::value_type, Size> result;
    boost::range::copy(cont, result.begin());
    return result;
}

bool does_file_exist(const std::string& path) {
    struct stat buffer;   
    return (stat(path.c_str(), &buffer) == 0); 
}

std::string SplitCam::get_serial()
{
    return _prefix + "_" + std::to_string(_index);
}

bool SplitCam::load_camera_info(std::string path)
{
    if (!does_file_exist(path)) {
        ROS_ERROR_STREAM("No camera info found. " << path << " doesn't exist. Not publishing camera info.");
        return false;
    }

    try {
        YAML::Node config = YAML::LoadFile(path);

        sensor_msgs::CameraInfo info;
        info.height = config["height"].as<int>();
        info.width = config["width"].as<int>();
        info.distortion_model = config["distortion_model"].as<std::string>("plumb_bob");
        info.D = config["D"].as<std::vector<double>>();
        info.K = as_array<9>(config["K"].as<std::vector<double>>());
        info.R = as_array<9>(config["R"].as<std::vector<double>>());
        info.P = as_array<12>(config["P"].as<std::vector<double>>());
        info.binning_x = config["binning_x"].as<int>(0);
        info.binning_y = config["binning_y"].as<int>(0);

        if (config["roi"]) {
            sensor_msgs::RegionOfInterest roi;
            roi.x_offset = config["roi"]["x_offset"].as<int>(0);
            roi.y_offset = config["roi"]["y_offset"].as<int>(0);
            roi.height = config["roi"]["height"].as<int>(0);
            roi.width = config["roi"]["width"].as<int>(0);
            roi.do_rectify = config["roi"]["do_rectify"].as<bool>(false);
            info.roi = roi;
        }
        _info = info;
    }
    catch (const YAML::BadFile& e) {
        std::cerr << e.msg << std::endl;
        return false;
    }
    catch (const YAML::ParserException& e) {
        std::cerr << e.msg << std::endl;
        return false;
    }
    
    return true;
}

void SplitCam::publish(cv::Mat image, ros::Time timestamp)
{
    std_msgs::Header header;
    header.frame_id = get_serial();
    header.stamp = timestamp;
    _info.header = header;

    if (_pub.getNumSubscribers() > 0) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
        _pub.publish(*msg, _info);
    }
}

SplitCam::~SplitCam()
{
    
}

TJ2Northstar::TJ2Northstar(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh)
{
    ros::param::param<std::string>("~topic_prefix", _prefix, "camera");

    std::string info_directory;
    ros::param::param<std::string>("~info_directory", info_directory, ".");

    int device_num;
    ros::param::param<int>("~device_num", device_num, 0);

    std::string fourcc_code;
    ros::param::param<std::string>("~fourcc_code", fourcc_code, "GREY");  // "GREY", "Y16"

    int width;
    ros::param::param<int>("~width", width, -1);

    int height;
    ros::param::param<int>("~height", height, -1);

    _arducam = new Arducam(device_num, fourcc_code, width, height);

    _combined_pub = _image_transport.advertiseCamera(_prefix + "/combined", 1);

    for (int index = 0; index < NUM_CAMERAS; index++) {
        cameras[index] = new SplitCam(info_directory, _prefix, index, nodehandle, &_image_transport);
    }
}

TJ2Northstar::~TJ2Northstar()
{
    delete _arducam;
}

int TJ2Northstar::run() {
    _arducam->start();
    while (ros::ok()) {
        ros::spinOnce();
        cv::Mat image;
        ros::Time now = ros::Time::now();
        if (!_arducam->read(image)) {
            ROS_ERROR("Failed to read from arducam. Re-opening.");
            _arducam->stop();
            ros::Duration(0.25).sleep();
            _arducam->start();
            ros::Duration(0.25).sleep();
            continue;
        }
        if (_combined_pub.getNumSubscribers() > 0) {
            sensor_msgs::ImagePtr combined_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
            _combined_pub.publish(*combined_msg, sensor_msgs::CameraInfo());
        }

        cv::Size size = image.size();
        int sub_width = size.width / NUM_CAMERAS;
        for (int index = 0; index < NUM_CAMERAS; index++) {
            cv::Rect split(sub_width * index, 0, sub_width, size.height);
            cameras[index]->publish(image(split), now);
        }
    }
    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_northstar");
    ros::NodeHandle nh;
    TJ2Northstar node(&nh);
    return node.run();
}
