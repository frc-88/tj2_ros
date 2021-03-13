# source: https://answers.ros.org/question/347754/jetson-nano-comes-with-opencv-411-do-i-need-to-downgrade-to-32-for-melodic/
sudo apt -y --allow-downgrades install libopencv-dev=3.2.0+dfsg-4ubuntu0.1
sudo apt-mark hold libopencv-dev
