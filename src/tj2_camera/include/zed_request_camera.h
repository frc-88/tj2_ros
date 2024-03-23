#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

class ZedRequestCamera
{
private:
    sl::Camera *_zed;
    sl::InitParameters *_init_parameters;

    cv::Mat poll();

public:
    ZedRequestCamera();
};
