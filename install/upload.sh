BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2
LOCAL_PATH=${PARENT_DIR}
DESTINATION_PATH=/home/tj2

if [ -z ${DESTINATION_NAME} ]; then
    echo "Please set a destination IP or hostname"
    exit
fi

if [ -z ${REMOTE_KEY} ]; then
    echo "Please set an SSH key file"
    exit
fi

rsync -avur --exclude-from=${LOCAL_PATH}/install/exclude.txt  -e "ssh -i ${REMOTE_KEY}"  ${LOCAL_PATH} tj2@${DESTINATION_NAME}:${DESTINATION_PATH}

SSH_COMMAND="ssh -i ${REMOTE_KEY} tj2@${DESTINATION_NAME}"

# restart systemd
echo "Restart roslaunch.service? (Y/n) "
read response
case $response in
  ([Nn])     echo "Skipping restart";;
  (*)        echo "Restarting roslaunch." && ${SSH_COMMAND} -t "sudo systemctl restart roslaunch.service";;
esac
