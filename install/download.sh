BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2
DOWNLOAD_PATH=$3

REMOTE_PATH=/home/tj2/tj2_ros/${DOWNLOAD_PATH}
LOCAL_PATH=${PARENT_DIR}/${DOWNLOAD_PATH}

if [ -z ${DESTINATION_NAME} ]; then
    echo "Please set a destination IP or hostname"
    exit
fi

if [ -z ${REMOTE_KEY} ]; then
    echo "Please set an SSH key file"
    exit
fi

scp -r -i ${REMOTE_KEY} -p 5810 tj2@${DESTINATION_NAME}:${REMOTE_PATH} ${LOCAL_PATH}
