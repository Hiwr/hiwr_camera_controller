cmake_minimum_required(VERSION 2.8.3)
project(hiwr_camera_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rospack
  dynamic_reconfigure
  sensor_msgs
  image_transport
  tf
  camera_info_manager
  driver_base
  nodelet
  hyve_msg
  )


generate_dynamic_reconfigure_options(
cfg/UVCCam.cfg
)

catkin_package(
   LIBRARIES hiwr_camera_controller display_nodelet
   DEPENDS roscpp nodelet std_msgs
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#install(DIRECTORY include/
#   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN ".h" PATTERN ".hpp"
#)


 install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

install(FILES hiwr_camera_controller.xml launch/camera.launch param/config_default.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include cfg/cpp ${catkin_INCLUDE_DIRS})


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -std=c++0x -pthread") 


add_library(hiwr_camera_controller src/hiwr_camera_controller.cpp src/uvc_cam.cpp )
target_link_libraries(hiwr_camera_controller ${catkin_LIBRARIES})

add_dependencies(hiwr_camera_controller  ${catkin_EXPORTED_TARGETS})


add_library(display_nodelet src/display_node.cpp)
target_link_libraries(display_nodelet ${catkin_LIBRARIES})

add_dependencies(display_nodelet  ${catkin_EXPORTED_TARGETS})
