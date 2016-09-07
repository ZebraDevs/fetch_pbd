# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(fetch_pbd_interaction)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  message_generation
  fetch_arm_control
  rospy
  rospy_message_converter
  std_msgs
  tf
)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# Generate messages and services
add_message_files(
  FILES
  ArmState.msg
  Condition.msg
  GripperAction.msg
  Landmark.msg
  ExecutionStatus.msg
  RobotSound.msg
  SessionState.msg
  WorldState.msg
  OrientationRPY.msg
)

add_service_files(
  FILES
  ExecuteActionById.srv
  GetSessionState.srv
  Ping.srv
  MoveArm.srv
  MoveArmTraj.srv
  SetGripperState.srv
  GetGripperState.srv
  GetEEPose.srv
  GetJointStates.srv
  GetArmMovement.srv
  GetGazeGoal.srv
  GetMostSimilarObject.srv
  GetObjectList.srv
  GetNearestObject.srv
  GetObjectFromName.srv
  GuiInput.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  tf
  fetch_arm_control
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
  INCLUDE_DIRS src/fetch_pbd_interaction
  LIBRARIES fetch_pbd_interaction
  CATKIN_DEPENDS rospy
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(pr2_pbd
#   src/${PROJECT_NAME}/pr2_pbd.cpp
# )

## Declare a cpp executable
# add_executable(pr2_pbd_node src/pr2_pbd_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pr2_pbd_node pr2_pbd_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(pr2_pbd_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS pr2_pbd_interaction pr2_pbd_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(PROGRAMS
  nodes/pbd_arm_control_node.py
  nodes/pbd_interaction_node.py
  nodes/pbd_world_node.py
  nodes/demo.py
  scripts/serve_web_interface.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/pbd.launch
  launch/pbd_prereqs.launch
  launch/pbd_rviz_viewer.launch
  launch/pbd_web_prereqs.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
  config/fetch_pbd.rviz
  config/zones.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY
  web_interface/fetch-pbd-gui
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/web_interface
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY
  examples
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY
  sounds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  web_interface/fetch-pbd-gui/install_web_dependencies.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/web_interface/fetch-pbd-gui
)

add_custom_target(install_web_dependencies ALL
  COMMAND "./install_web_dependencies.sh"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/web_interface/fetch-pbd-gui)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pr2_pbd.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
