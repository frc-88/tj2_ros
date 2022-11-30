
#include "tj2_video.h"

TJ2Video::TJ2Video(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh)
{

}

int TJ2Video::run()
{
    ros::spin();
    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_video");
    ros::NodeHandle nh;

    TJ2Video node(&nh);
    int err = node.run();

    return err;
}
