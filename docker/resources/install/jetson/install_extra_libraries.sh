# pyuvc
cd /tmp
git clone https://github.com/pupil-labs/pyuvc --recursive
cd pyuvc
git checkout v1.0.0rc1
cd ..
export FORCE_LOCAL_LIBUVC_BUILD=ON
python -m pip install ./pyuvc
