BASE_DIR=$(realpath "$(dirname $0)")
PARENT_DIR=$(dirname $BASE_DIR)
DESTINATION_NAME=$1
REMOTE_KEY=$2
RESTART_ROSLAUNCH=$3

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
echo "Stopping roslaunch"
${SSH_COMMAND} -t "sudo systemctl stop roslaunch.service"

# build tj2_tools
${SSH_COMMAND} "cd ${DEST_FULL_PATH}/tj2_tools && python3 setup.py -q install --user"

# build catkin ws

cd ${PACKAGES_PATH}
PACKAGE_LIST=`ls -d tj2_* | sed 's/\///g'`
PACKAGE_LIST=`echo "$PACKAGE_LIST" | tr '\n' ';'`
cd -

${SSH_COMMAND} -t "export OPENBLAS_CORETYPE=ARMV8 && cd ${CATKIN_WS_PATH} && source /home/${USERNAME}/noetic_ws/install_isolated/setup.bash && source ${CATKIN_WS_PATH}/devel/setup.bash && catkin_make -DCATKIN_WHITELIST_PACKAGES='$PACKAGE_LIST'"

${BASE_DIR}/restart.sh ${DESTINATION_NAME} ${REMOTE_KEY} ${RESTART_ROSLAUNCH}
