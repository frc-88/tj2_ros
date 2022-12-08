BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2
RESTART_SERVICE=$3

LOCAL_PATH=${PARENT_DIR}
DESTINATION_PATH=/home/tj2
CATKIN_WS_PATH=/home/tj2/ros_ws

if [ -z ${DESTINATION_NAME} ]; then
    echo "Please set a destination IP or hostname"
    exit
fi

if [ -z ${REMOTE_KEY} ]; then
    echo "Please set an SSH key file"
    exit
fi

LOCAL_PATH=$(realpath $LOCAL_PATH)
LOCAL_NAME=$(basename $LOCAL_PATH)
DEST_FULL_PATH=${DESTINATION_PATH}/${LOCAL_NAME}
PACKAGES_PATH=${LOCAL_PATH}

${BASE_DIR}/upload.sh ${DESTINATION_NAME} ${REMOTE_KEY} n

SSH_COMMAND="ssh -i ${REMOTE_KEY} -p 5810 tj2@${DESTINATION_NAME}"

# stop roslaunch
echo "Stopping tj2_ros"
${SSH_COMMAND} -t "sudo systemctl stop tj2_ros.service"

# build tj2_ros
${SSH_COMMAND} "cd ${DEST_FULL_PATH}/docker/native/build_tj2_ros.sh"

${BASE_DIR}/restart.sh ${DESTINATION_NAME} ${REMOTE_KEY} ${RESTART_SERVICE} tj2_ros
