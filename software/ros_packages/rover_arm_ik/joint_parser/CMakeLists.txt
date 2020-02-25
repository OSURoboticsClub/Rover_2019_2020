Cmake_minimum_required(VERSION 2.8.3)
project(joint_parser)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   message_generation
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
   message_generation
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(joint_remapper src/joint_remapper.cpp)

target_link_libraries(joint_remapper
   ${catkin_LIBRARIES}
)
