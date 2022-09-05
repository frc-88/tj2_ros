BASE_DIR=$(realpath "$(dirname $0)")

echo "---"
echo "Installing dependencies via python pip"
echo "---"

cd ${BASE_DIR}/../source_installation

pip3 install -r requirements.txt

cd ${BASE_DIR}
bash install_python_libraries.sh
