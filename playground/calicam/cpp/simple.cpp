
// This is No Warranty No Copyright Software.
// astar.ai
// Nov 16, 2018

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

int main(int argc, char** argv) {
  int cap_num = argc == 2 ? atoi(argv[1]) : 0;
  cv::Mat raw_img;
  cv::VideoCapture vcapture;
  vcapture.open(cap_num);

  if (!vcapture.isOpened()) {
    std::cout << "Camera doesn't work" << std::endl;
    exit(-1);
  }

  vcapture.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
  vcapture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  vcapture.set(cv::CAP_PROP_FPS, 30);
  int frame_count = 0;

  std::string param_win_name("calicam");
  cv::namedWindow(param_win_name);

  cv::Mat image;
  while (true) {
    vcapture >> image;
    frame_count++;
    std::cout << "frame #" << frame_count << std::endl;

    if (image.total() == 0) {
      std::cout << "Image capture error" << std::endl;
      exit(-1);
    }

    imshow(param_win_name, image);

    char key = cv::waitKey(1);
    if (key == 'q' || key == 'Q' || key == 27)
      break;
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////



