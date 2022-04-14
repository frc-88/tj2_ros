cmake_minimum_required(VERSION 3.0)
project(tj2_target)

find_package(catkin REQUIRED COMPONENTS
    rospy
    # geometry_msgs
    std_msgs
    message_generation
    genmsg
    # actionlib_msgs
    # actionlib
)

catkin_python_setup() # added if there is a setup.py. In this case for the node

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RevColorSensor.msg
  TargetConfig.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    SetProbability.srv
    SetProbabilityRing.srv
    RecordValue.srv
)

# add_action_files(
#     DIRECTORY action
#     FILES
# )

generate_messages(
    DEPENDENCIES
    std_msgs
    # geometry_msgs
    # actionlib_msgs
)


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
    CATKIN_DEPENDS rospy std_msgs message_runtime
)

# catkin_install_python(PROGRAMS src/tj2_target/central_planning.py
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# add_library(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.py)
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.py)
# add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE py)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE CMAKE_py_LINK_EXECUTABLE)
