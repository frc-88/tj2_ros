cmake_minimum_required(VERSION 2.8.3)
project(tj2_match_watcher)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
)

catkin_python_setup() # added if there is a setup.py. In this case for the node


## Generate services in the 'srv' folder
# add_service_files(
#     FILES
# )

# add_action_files(
#     DIRECTORY action
#     FILES

# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs
#     geometry_msgs
#     actionlib_msgs
# )


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# generate_dynamic_reconfigure_options(
#     
# )

catkin_package(
    CATKIN_DEPENDS rospy std_msgs
)

# catkin_install_python(PROGRAMS src/tj2_match_watcher/central_planning.py
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# add_library(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.py)
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.py)
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE py)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE CMAKE_py_LINK_EXECUTABLE)
