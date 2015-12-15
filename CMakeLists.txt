cmake_minimum_required(VERSION 2.8.3)
project(skeleton_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  visualization_msgs
  image_transport
  kdl_conversions
  openni2_camera
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  tf
  message_generation
)

#find_package(opencv REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_message_files(
   FILES
   user_IDs.msg
   skeleton_complete.msg
   skeleton_tracker_state.msg
)

generate_messages(
    DEPENDENCIES geometry_msgs visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES skeleton_tracker
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport kdl_conversions openni2_camera pcl_conversions pcl_ros roscpp roslib tf visualization_msgs
  DEPENDS system_lib opencv message_runtime
)


set(OPENNI2_WRAPPER /opt/ros/indigo/include/openni2_camera/)
set(OPENNI2_DIR ~/OpenNI2)
set(NITE2_DIR ~/NiTE-2.0.0/)
set(NITE2_LIB ~/NiTE-2.0.0/Redist/libNiTE2.so)
link_directories(${NITE2_DIR}/Redist)
include_directories(${NITE2_DIR}/Include)

link_directories(${OPENNI2_DIR}/Bin/x64-Release)

include_directories(${OPENNI2_DIR}/Bin/x64-Release)
include_directories(/usr/include/openni2/)
include_directories(${OPENNI2_DIR}/Include)
include_directories(${OPENNI2_WRAPPER})
include_directories(${NITE2_DIR}/Include)
include_directories(${OpenCV_INCLUDE_DIRS}/include)

add_executable(xtion_tracker src/xtion_tracker_node.cpp)
target_link_libraries(xtion_tracker OpenNI2 ${NITE2_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
#add_executable(skeleton_tracker src/skeleton_tracker_node.cpp)
#target_link_libraries(skeleton_tracker OpenNI2 NiTE2 ${NITE2_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
#add_executable(tracker_publisher src/tracker_publisher.cpp)
#target_link_libraries(tracker_publisher OpenNI2 ${NITE2_LIB} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
