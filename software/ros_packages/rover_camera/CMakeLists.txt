cmake_minimum_required(VERSION 2.8)
project(rover_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        message_generation
        sensor_msgs
        std_msgs
)

# add the resized image message
add_message_files(
        FILES
        CameraControlMessage.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add the publisher example
add_executable(rover_camera src/rover_camera.cpp)
add_dependencies(rover_camera ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(rover_camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# Mark executables and/or libraries for installation
install(TARGETS rover_camera
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(FILES resized_plugins.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
