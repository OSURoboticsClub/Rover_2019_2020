cmake_minimum_required(VERSION 2.8.3)
project(rover_arm_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## C++ executable
add_executable(jointState_converter src/jointState_converter.cpp)
#add_executable(rover_arm_moveit_config src/moveit_interface.cpp)
