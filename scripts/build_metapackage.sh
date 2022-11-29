BASE_DIR=$(realpath "$(dirname $0)")
FOLDER_NAME=$1

CATKIN_WS_PATH=~/ros_ws
USERNAME=tj2

cd ${CATKIN_WS_PATH}/src/${FOLDER_NAME}

PACKAGE_LIST=`ls -d */ | sed 's/\///g'`
PACKAGE_LIST=`echo "$PACKAGE_LIST" | tr '\n' ';'`
echo $PACKAGE_LIST

cd ${CATKIN_WS_PATH}

export OPENBLAS_CORETYPE=ARMV8
source /home/${USERNAME}/noetic_ws/install_isolated/setup.bash
source ${CATKIN_WS_PATH}/devel/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="$PACKAGE_LIST"
