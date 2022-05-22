cmake_minimum_required(VERSION 2.8)
project(stereo_calicam LANGUAGES C CXX)

find_package(OpenCV REQUIRED)

add_executable(stereo_calicam stereo_calicam.cpp)
add_executable(simple simple.cpp)
target_link_libraries(simple ${OpenCV_LIBRARIES})
target_link_libraries(stereo_calicam ${OpenCV_LIBRARIES})
