#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include "opencv2/ximgproc/disparity_filter.hpp"

class StereoCalicam
{
private:
    cv::VideoCapture capture;
    int capture_num;
    
    int size_setting;
    std::string cam_model;
    
    std::string param_path;
    
    int capture_width, capture_height;
    int image_width, image_height, image_channels;
    int config_cap_width, config_cap_height, config_image_width, config_image_height;

    cv::Mat raw, raw_left, raw_right, rect_left, rect_right, disparity, depth;

    cv::Ptr<cv::StereoSGBM> stereo;
    cv::Ptr<cv::StereoMatcher> right_matcher;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
    int num_disparities;
    int disparity_window;
    int max_x_grad;
    int smoothing_factor;
    double wls_lambda;
    double wls_sigma;

    double virtual_fov;
    cv::Mat Translation, Kl, Kr, Dl, Dr, xil, xir, Rl, Rr, smap[2][2], Knew;

    void init_undistort_rectify_map(cv::Mat K, cv::Mat D, cv::Mat xi, cv::Mat R,
                                    cv::Mat P, cv::Size size,
                                    cv::Mat &map1, cv::Mat &map2);

    void init_rectify_map();
    void load_parameters(std::string file_name);

public:
    StereoCalicam(int capture_num, std::string param_path, int size_setting, double virtual_fov);
    void begin();
    cv::Mat get_raw() { return raw; }
    cv::Mat get_left_raw() { return raw_left; }
    cv::Mat get_right_raw() { return raw_right; }
    cv::Mat get_left_rect() { return rect_left; }
    cv::Mat get_right_rect() { return rect_right; }
    cv::Mat get_disparity() { return disparity; }
    cv::Mat get_depth();
    void process();

    ~StereoCalicam();
};

inline double MatRowMul(cv::Mat m, double x, double y, double z, int r)
{
    return m.at<double>(r, 0) * x + m.at<double>(r, 1) * y + m.at<double>(r, 2) * z;
}

StereoCalicam::StereoCalicam(int capture_num, std::string param_path, int size_setting, double virtual_fov)
{
    if (size_setting == 0)
    {
        capture_width = 2560;
        capture_height = 960;
    }
    else
    {
        capture_width = 1280;
        capture_height = 480;
    }
    image_channels = 3;
    image_width = capture_width / 2;
    image_height = capture_height;
    this->virtual_fov = virtual_fov;
    this->capture_num = capture_num;
    this->param_path = param_path;
}

void StereoCalicam::begin()
{
    load_parameters(param_path);
    init_rectify_map();

    capture.open(capture_num);
    if (!capture.isOpened())
    {
        std::cout << "Camera doesn't work" << std::endl;
        exit(-1);
    }

    capture.set(cv::CAP_PROP_FRAME_WIDTH, capture_width);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, capture_height);
    capture.set(cv::CAP_PROP_FPS, 30);

    int N = num_disparities;
    int W = disparity_window;
    int C = image_channels;
    stereo = cv::StereoSGBM::create(0, N, W, 8 * C * W * W, 32 * C * W * W);
    stereo->setP1(24 * W * W * smoothing_factor);
    stereo->setP2(96 * W * W * smoothing_factor);
    stereo->setPreFilterCap(max_x_grad);
    stereo->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    right_matcher = cv::ximgproc::createRightMatcher(stereo);
    wls_filter = cv::ximgproc::createDisparityWLSFilter(stereo);
    wls_filter->setLambda(wls_lambda);
    wls_filter->setSigmaColor(wls_sigma);
}

void StereoCalicam::load_parameters(std::string file_name)
{
    cv::FileStorage fs(file_name, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "Failed to open ini parameters: " << file_name << std::endl;
        exit(-1);
    }

    cv::Size cap_size;
    fs["cam_model"] >> cam_model;
    fs["cap_size"] >> cap_size;
    fs["Kl"] >> Kl;
    fs["Dl"] >> Dl;
    fs["xil"] >> xil;
    Rl = cv::Mat::eye(3, 3, CV_64F);
    fs["Rl"] >> Rl;
    fs["Kr"] >> Kr;
    fs["Dr"] >> Dr;
    fs["xir"] >> xir;
    fs["Rr"] >> Rr;
    fs["T"] >> Translation;
    fs["num_disparities"] >> num_disparities;
    fs["disparity_window"] >> disparity_window;
    fs["max_x_grad"] >> max_x_grad;
    fs["smoothing_factor"] >> smoothing_factor;
    fs["wls_lambda"] >> wls_lambda;
    fs["wls_sigma"] >> wls_sigma;
    fs.release();

    config_image_width = cap_size.width / 2;
    config_image_height = cap_size.height;
    config_cap_width = cap_size.width;
    config_cap_height = cap_size.height;
}

void StereoCalicam::init_undistort_rectify_map(cv::Mat K, cv::Mat D, cv::Mat xi, cv::Mat R,
                                               cv::Mat P, cv::Size size,
                                               cv::Mat &map1, cv::Mat &map2)
{
    map1 = cv::Mat(size, CV_32F);
    map2 = cv::Mat(size, CV_32F);

    double fx = K.at<double>(0, 0);
    double fy = K.at<double>(1, 1);
    double cx = K.at<double>(0, 2);
    double cy = K.at<double>(1, 2);
    double s = K.at<double>(0, 1);

    double xid = xi.at<double>(0, 0);

    double k1 = D.at<double>(0, 0);
    double k2 = D.at<double>(0, 1);
    double p1 = D.at<double>(0, 2);
    double p2 = D.at<double>(0, 3);

    cv::Mat KRi = (P * R).inv();

    for (int r = 0; r < size.height; ++r)
    {
        for (int c = 0; c < size.width; ++c)
        {
            double xc = MatRowMul(KRi, c, r, 1., 0);
            double yc = MatRowMul(KRi, c, r, 1., 1);
            double zc = MatRowMul(KRi, c, r, 1., 2);

            double rr = sqrt(xc * xc + yc * yc + zc * zc);
            double xs = xc / rr;
            double ys = yc / rr;
            double zs = zc / rr;

            double xu = xs / (zs + xid);
            double yu = ys / (zs + xid);

            double r2 = xu * xu + yu * yu;
            double r4 = r2 * r2;
            double xd = (1 + k1 * r2 + k2 * r4) * xu + 2 * p1 * xu * yu + p2 * (r2 + 2 * xu * xu);
            double yd = (1 + k1 * r2 + k2 * r4) * yu + 2 * p2 * xu * yu + p1 * (r2 + 2 * yu * yu);

            double u = fx * xd + s * yd + cx;
            double v = fy * yd + cy;

            map1.at<float>(r, c) = (float)u;
            map2.at<float>(r, c) = (float)v;
        }
    }
}

void StereoCalicam::init_rectify_map()
{
    double vfov_rad = virtual_fov * CV_PI / 180.;
    double focal = config_image_height / 2. / tan(vfov_rad / 2.);
    Knew = (cv::Mat_<double>(3, 3) << focal, 0., config_image_width / 2. - 0.5,
            0., focal, config_image_height / 2. - 0.5,
            0., 0., 1.);

    cv::Size img_size(config_image_width, config_image_height);

    init_undistort_rectify_map(Kl, Dl, xil, Rl, Knew,
                               img_size, smap[0][0], smap[0][1]);

    std::cout << "Width: " << config_image_width << "\t"
              << "Height: " << config_image_height << "\t"
              << "V.Fov: " << virtual_fov << "\n";
    std::cout << "K Matrix: \n"
              << Knew << std::endl;

    init_undistort_rectify_map(Kr, Dr, xir, Rr, Knew,
                               img_size, smap[1][0], smap[1][1]);
    std::cout << "Ndisp: " << num_disparities << "\t"
              << "Wsize: " << disparity_window << "\n";
    std::cout << std::endl;
}

void StereoCalicam::process()
{
    cv::Mat raw_unsized;
    capture >> raw_unsized;
    cv::resize(raw_unsized, raw, cv::Size(config_cap_width, config_cap_height), cv::INTER_NEAREST);
    raw(cv::Rect(0, 0, config_image_width, config_cap_height)).copyTo(raw_left);
    raw(cv::Rect(config_image_width, 0, config_image_width, config_cap_height)).copyTo(raw_right);

    cv::remap(raw_left, rect_left, smap[0][0], smap[0][1], 1, 0);
    cv::remap(raw_right, rect_right, smap[1][0], smap[1][1], 1, 0);

    cv::resize(rect_left, rect_left, cv::Size(), 0.5, 0.5);
    cv::resize(rect_right, rect_right, cv::Size(), 0.5, 0.5);
    cv::Mat left_disparity, right_disparity;
    stereo->compute(rect_left, rect_right, left_disparity);
    right_matcher->compute(rect_right, rect_left, right_disparity);
    wls_filter->filter(left_disparity, rect_left, disparity, right_disparity);
}

cv::Mat StereoCalicam::get_depth()
{
    double fx = Knew.at<double>(0, 0);
    double fy = Knew.at<double>(1, 1);
    double cx = Knew.at<double>(0, 2);
    double cy = Knew.at<double>(1, 2);
    double bl = -Translation.at<double>(0, 0);

    cv::Mat dispf;
    disparity.convertTo(dispf, CV_32F, 1.f / 16.f);
    for (int r = 0; r < dispf.rows; ++r)
    {
        for (int c = 0; c < dispf.cols; ++c)
        {
            double e = (c - cx) / fx;
            double f = (r - cy) / fy;

            double disp = dispf.at<float>(r, c);
            if (disp <= 0.f)
                continue;

            double depth = fx * bl / disp;
            double x = e * depth;
            double y = f * depth;
            double z = depth;
        }
    }
    return dispf; // TODO return point cloud
}

StereoCalicam::~StereoCalicam()
{
    capture.release();
}

int main(int argc, char **argv)
{
    int cap_num = argc >= 2 ? atoi(argv[1]) : 0;
    std::string param_path = argc >= 3 ? argv[2] : "./astar_calicam.yml";
    int vfov = argc >= 4 ? atoi(argv[3]) : 120;
    int size_setting = argc >= 5 ? atoi(argv[4]) : 0;
    cv::Mat raw_img;
    StereoCalicam cam(cap_num, param_path, size_setting, (double)vfov);
    cam.begin();

    std::string param_win_name("calicam");
    cv::namedWindow(param_win_name);

    cv::Mat image, disp_color;
    while (true)
    {
        cam.process();

        if (cam.get_raw().total() == 0)
        {
            std::cout << "Image capture error" << std::endl;
            exit(-1);
        }

        double minVal, maxVal;
        cv::Mat disp16s = cam.get_disparity();
        cv::minMaxLoc(disp16s, &minVal, &maxVal);
        disp16s.convertTo(disp_color, CV_8UC1, 255 / (maxVal - minVal));
        cv::cvtColor(disp_color, disp_color, cv::COLOR_GRAY2BGR);
        cv::vconcat(cam.get_left_rect(), disp_color, image);

        imshow(param_win_name, image);

        char key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27)
            break;
    }

    return 0;
}
