#include "tj2_yolo.h"

TJ2Yolo::TJ2Yolo(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<std::string>("~model_path", _model_path, "best.pt");
    ros::param::param<std::string>("~classes_path", _classes_path, "coco.names");
    ros::param::param<std::string>("~model_path", _model_path, "best.pt");
    ros::param::param<float>("~confidence_threshold", _conf_threshold, 0.25);
    ros::param::param<float>("~model_path", _iou_threshold, 0.45);

    ros::param::param<std::string>("~image_width_param", _image_width_param, "/camera/realsense2_camera/color_width");
    ros::param::param<std::string>("~image_height_param", _image_height_param, "/camera/realsense2_camera/color_height");
    ros::param::param<int>(_image_width_param, _image_width, 960);
    ros::param::param<int>(_image_height_param, _image_height, 540);

    torch::DeviceType device_type;
    if (torch::cuda::is_available()) {
        device_type = torch::kCUDA;
    } else {
        device_type = torch::kCPU;
    }
    class_names = Detector::LoadNames(_classes_path);
    if (class_names.empty()) {
        std::cerr << "Error loading class names!\n";
        std::exit(EXIT_FAILURE);
    }

    detector = new Detector(_model_path, device_type);
    // run once to warm up
    auto temp_img = cv::Mat::zeros(_image_height, _image_width, CV_32FC3);
    detector->Run(temp_img, 1.0f, 1.0f);
}


int TJ2Yolo::run()
{
    ros::spin();

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_yolo");
    ros::NodeHandle nh;
    TJ2Yolo node(&nh);
    return node.run();
}
