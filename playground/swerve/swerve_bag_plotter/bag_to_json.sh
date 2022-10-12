BASE_DIR=$(realpath "$(dirname $0)")
BAG_PATH=$1

CONVERT_DIR=${BASE_DIR}/../../../tj2_tools/tj2_tools/rosbag_to_file/
OUTPUT_DIR=${BASE_DIR}/data

cd ${CONVERT_DIR}

python3 convert.py -a -p ${BAG_PATH} --type=json -o ${OUTPUT_DIR}
