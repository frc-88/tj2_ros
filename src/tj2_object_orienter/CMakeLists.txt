cmake_minimum_required(VERSION 3.0.2)
project(tj2_object_orienter)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp
    tf2
    tf2_ros
)

find_package(Open3D REQUIRED)

catkin_python_setup()


catkin_package(
    INCLUDE_DIRS include
    # LIBRARIES tj2_object_orienter
    CATKIN_DEPENDS
        rospy
        tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

add_executable(tj2_object_orienter
  src/tj2_object_orienter.cpp
)

target_link_libraries(tj2_object_orienter
    ${catkin_LIBRARIES}
    Open3D::Open3D
)

add_dependencies(tj2_object_orienter ${tj2_object_orienter_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS tj2_object_orienter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
