cmake_minimum_required(VERSION 2.8.3)
project(stag_ros)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  image_transport
  cv_bridge
  ar_track_alvar_msgs
)

catkin_package(
    CATKIN_DEPENDS std_msgs roscpp image_transport image_transport
)

include_directories(
  "src/"
  ${catkin_INCLUDE_DIRS}
  "/opt/ros/melodic/include"
  ${OpenCV_INCLUDE_DIRS}
)

FILE(GLOB stag_srcs src/*.cpp)
FILE(GLOB ED_srcs src/ED/*.cpp)

LIST(APPEND stag_srcs ${ED_srcs})

add_library(stag_lib ${stag_srcs})
target_link_libraries(stag_lib ${OpenCV_LIBS})

add_executable(stag_ros_node nodes/stagNode.cpp)
target_link_libraries(stag_ros_node 
    ${catkin_LIBRARIES}
    stag_lib 
    ${OpenCV_LIBS}
)

install(TARGETS stag_ros_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
