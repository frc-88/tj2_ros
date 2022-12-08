BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2
RESTART_ROSLAUNCH=$3

LOCAL_PATH=${PARENT_DIR}
DESTINATION_PATH=/home/tj2
LOCAL_NAME=$(basename $LOCAL_PATH)
DESTINATION_PATH=/home/tj2
DEST_FULL_PATH=${DESTINATION_PATH}/${LOCAL_NAME}

if [ -z ${DESTINATION_NAME} ]; then
    echo "Please set a destination IP or hostname"
    exit
fi

if [ -z ${REMOTE_KEY} ]; then
    echo "Please set an SSH key file"
    exit
fi

SSH_COMMAND="ssh -i ${REMOTE_KEY} -p 5810 tj2@${DESTINATION_NAME}"

OUTPUT=$( rsync -avur --exclude-from=${LOCAL_PATH}/install/exclude.txt  -e "ssh -i ${REMOTE_KEY} -p 5810"  ${LOCAL_PATH} tj2@${DESTINATION_NAME}:${DESTINATION_PATH} | tee /dev/tty)

if echo "$OUTPUT" | grep -q 'tj2_tools/'; then
    # build tj2_tools
    ${SSH_COMMAND} "cd ${DEST_FULL_PATH}/docker/native/build_tj2_tools.sh"
fi

${BASE_DIR}/restart.sh ${DESTINATION_NAME} ${REMOTE_KEY} ${RESTART_ROSLAUNCH}
