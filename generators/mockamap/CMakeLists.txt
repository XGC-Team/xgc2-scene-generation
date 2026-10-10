cmake_minimum_required(VERSION 3.0.2)
project(mockamap)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_SRCS
  src/mockamap.cpp
  src/maps.cpp
  src/perlinnoise.cpp
)

add_executable(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_SRCS}
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
