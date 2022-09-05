BASE_DIR=$(realpath "$(dirname $0)")

BUILD_WS=$HOME/build_ws

mkdir -p ${BUILD_WS}

cd ${BUILD_WS}
git clone https://github.com/FRC-Utilities/LibDS.git
cd LibDS
qmake -qt=qt5
make -j5
sudo make install

cd ${BUILD_WS}
git clone --recursive https://github.com/ThadHouse/CmakeWpilib.git
cd CmakeWpilib
mkdir build
cd build
cmake .. -DWITHOUT_CSCORE=ON -DWITHOUT_JAVA=ON
cd ../libraries/wpiutil/wpiutil/
cp ${BASE_DIR}/../source_installation/fix-pthread-related-bug.patch .
git apply fix-pthread-related-bug.patch --reject --whitespace=fix
cd ../../../build
make -j4
sudo make install
