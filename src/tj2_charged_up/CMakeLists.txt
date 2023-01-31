cmake_minimum_required(VERSION 3.0.2)
project(tj2_charged_up)

find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs
    std_msgs
    message_generation
    genmsg
    dynamic_reconfigure
)

catkin_python_setup()


generate_dynamic_reconfigure_options(
  dynamic_config/ParticleFilter.cfg
)

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES tj2_charged_up
    CATKIN_DEPENDS
      rospy
      std_msgs
      message_runtime
      dynamic_reconfigure
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
