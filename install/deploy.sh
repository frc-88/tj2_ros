BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2
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

LOCAL_NAME=$(basename $LOCAL_PATH)
DEST_FULL_PATH=${DESTINATION_PATH}/${LOCAL_NAME}

${BASE_DIR}/upload.sh ${DESTINATION_NAME} ${REMOTE_KEY} ${LOCAL_PATH} ${DESTINATION_PATH}

SSH_COMMAND="ssh -i ${REMOTE_KEY} tj2@${DESTINATION_NAME}"

# build tj2_tools
${SSH_COMMAND} "cd ${DEST_FULL_PATH}/tj2_tools && python3 setup.py -q install --user"

# build catkin ws
${SSH_COMMAND} -t "cd ${CATKIN_WS_PATH} && source ${CATKIN_WS_PATH}/devel/setup.bash && catkin_make"

