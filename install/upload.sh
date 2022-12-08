BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2
RESTART_SERVICE=$3
SERVICE_NAME={$4:-roslaunch}

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

if [ "${SERVICE_NAME}" == "roslaunch" ]; then
    if echo "$OUTPUT" | grep -q 'tj2_tools/'; then
        # build tj2_tools
        ${SSH_COMMAND} "cd ${DEST_FULL_PATH}/tj2_tools && python3 setup.py -q install --user"
    fi
fi

${BASE_DIR}/restart.sh ${DESTINATION_NAME} ${REMOTE_KEY} ${RESTART_SERVICE} ${SERVICE_NAME}
