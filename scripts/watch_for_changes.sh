BASE_DIR=$(realpath "$(dirname $0)")

while true; do
    find ${BASE_DIR}/.. -follow | entr ${BASE_DIR}/copy_tj2_ros.sh
done