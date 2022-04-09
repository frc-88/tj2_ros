BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2

LOCAL_PATH=${PARENT_DIR}
REMOTE_PATH=/home/tj2/tj2_ros/${DOWNLOAD_PATH}

if [ -z ${DESTINATION_NAME} ]; then
    echo "Please set a destination IP or hostname"
    exit
fi

if [ -z ${REMOTE_KEY} ]; then
    echo "Please set an SSH key file"
    exit
fi

rsync -avur --include-from=${LOCAL_PATH}/install/include.txt  -e "ssh -i ${REMOTE_KEY} -p 5810" tj2@${DESTINATION_NAME}:${DESTINATION_PATH} ${LOCAL_PATH}
