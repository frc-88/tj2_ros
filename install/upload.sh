BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2
LOCAL_PATH=${3:-${PARENT_DIR}}
DESTINATION_PATH=${4:-/home/tj2}

if [ -z ${DESTINATION_NAME} ]; then
    echo "Please set a destination IP or hostname"
    exit
fi

if [ -z ${REMOTE_KEY} ]; then
    echo "Please set an SSH key file"
    exit
fi

rsync -avur --delete --exclude-from=${LOCAL_PATH}/install/exclude.txt  -e "ssh -i ${REMOTE_KEY}"  ${LOCAL_PATH} tj2@${DESTINATION_NAME}:${DESTINATION_PATH}