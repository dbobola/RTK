cmake_minimum_required(VERSION 2.8.3)
project(ntrip_ros)

find_package(catkin REQUIRED COMPONENTS
#  rtcm_msgs
  message_generation
  message_runtime
  #nmea_msgs
  rospy
  std_msgs
)

add_message_files(
  FILES
  GnssPVTSolnMsg.msg
  GnssTimeMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YOUR_PACKAGE_NAME REQUIRED IMPORTED_TARGET rtklib)

# Add RTKLIB library and headers
include_directories(${YOUR_PACKAGE_NAME_INCLUDE_DIRS})
link_directories(${YOUR_PACKAGE_NAME_LIBRARY_DIRS})

# Link RTKLIB library with NTRIP_ROS package
target_link_libraries(your_ntrip_server_node ${YOUR_PACKAGE_NAME_LIBRARIES})

cmake_minimum_required(VERSION 2.8.3)
project(ntrip_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

# Add the RTKLIB library to the build process
find_path(RTKLIB_INCLUDE_DIRS rtklib.h PATH_SUFFIXES rtklib)
find_library(RTKLIB_LIBRARIES NAMES rtklib librtklib)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${RTKLIB_INCLUDE_DIRS}
)

add_executable(ntrip_client_node src/ntrip_client_node.cpp)

target_link_libraries(ntrip_client_node
  ${catkin_LIBRARIES}
  ${RTKLIB_LIBRARIES}
)
