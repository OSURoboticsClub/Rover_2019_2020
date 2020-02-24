Cmake_minimum_required(VERSION 2.8.3)
project(rover_arm_moveit_parser)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
)

add_message_files(
    FILES
    ArmJointStates.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(CATKIN_DEPENDS 
   roscpp 
   std_msgs 
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(joint_position src/parse_joint_states.cpp)

#target_link_libraries(
#   rover_arm_moveit_parser
#   ${catkin_LIBRARIES}
#)
