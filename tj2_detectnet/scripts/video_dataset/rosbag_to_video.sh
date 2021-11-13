
BAG_NAME=$1
BAG_PATH=/home/ben/Diff-Swerve-ROS/tj2_camera/bags/$BAG_NAME
OUT_NAME=`basename $BAG_PATH .bag`.mp4
./rosbag2video.py -v -o output/$OUT_NAME -t /camera/color/image_raw $BAG_PATH
