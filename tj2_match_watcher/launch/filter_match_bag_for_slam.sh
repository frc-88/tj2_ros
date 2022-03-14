BASE_DIR=$(realpath "$(dirname $0)")

rosbag filter $BASE_DIR/../bags/$1 $BASE_DIR/../bags/$2 "topic in ('/sinistra_laser/scan', '/dextra_laser/scan', '/tj2/odom', '/camera/color/image_raw')"
