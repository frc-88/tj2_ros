cmake_minimum_required(VERSION 3.0.2)
project(tj2_charged_up)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_python_setup()


catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES tj2_charged_up
    # CATKIN_DEPENDS
    #     rospy
    #     tf2
    #     sensor_msgs
)

# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )
# link_directories(${catkin_LIBRARY_DIRS})

# add_executable(tj2_charged_up
#   src/tj2_charged_up.cpp
# )

# target_link_libraries(tj2_charged_up
#     ${catkin_LIBRARIES}
# )

# add_dependencies(tj2_charged_up ${tj2_charged_up_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# install(TARGETS tj2_charged_up
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
