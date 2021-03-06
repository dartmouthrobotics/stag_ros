cmake_minimum_required(VERSION 2.8.3)
project(stag_ros)

find_package(OpenCV 3.2.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  image_transport
  cv_bridge
  nodelet
  message_generation
)

add_message_files(
  FILES
  StagMarkers.msg
  StagMarker.msg
)

add_service_files(
  FILES
  SetTrackedBundles.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs
)

catkin_package(
    LIBRARIES stag_ros stag_lib
    CATKIN_DEPENDS std_msgs roscpp image_transport image_transport nodelet std_msgs message_generation geometry_msgs
)

add_subdirectory(modules/stag)

include_directories(
  "modules/stag/src"
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_library(stag_ros nodes/StagNodelet.cpp)
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

install(TARGETS stag_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES stag_ros.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
