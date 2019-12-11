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
  nodelet
)

catkin_package(
    LIBRARIES stag_ros
    CATKIN_DEPENDS std_msgs roscpp image_transport image_transport nodelet
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

add_library(stag_ros nodes/stagNodelet.cpp)
target_link_libraries(stag_ros
    ${catkin_LIBRARIES}
    stag_lib 
    ${OpenCV_LIBS}
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(stag_ros ${catkin_EXPORTED_LIBRARIES})
endif()

install(TARGETS stag_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES stag_ros.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
