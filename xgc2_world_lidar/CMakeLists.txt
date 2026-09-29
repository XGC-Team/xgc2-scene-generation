cmake_minimum_required(VERSION 3.0.2)
project(xgc2_world_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## The core library and its tests have no ROS dependency. Without catkin
## (plain `cmake`) only they are built.
find_package(Eigen3 REQUIRED)
find_package(catkin QUIET COMPONENTS
  roscpp
  roslib
  geometry_msgs
  sensor_msgs
  std_srvs
  xgc2_geometry_msgs
)

if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp roslib geometry_msgs sensor_msgs std_srvs xgc2_geometry_msgs
  )
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

## ROS-free core, linked statically into the node and the tests
add_library(${PROJECT_NAME} STATIC
  src/world_lidar.cpp
  src/scene_conversion.cpp
  src/convex_body_conversion.cpp
)
if(TARGET Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
else()
  target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

## Plain-assert tests (no ROS, no gtest)
add_executable(test_world_lidar test/test_world_lidar.cpp)
target_link_libraries(test_world_lidar PRIVATE ${PROJECT_NAME})
add_executable(test_scene_sources test/test_scene_sources.cpp)
target_link_libraries(test_scene_sources PRIVATE ${PROJECT_NAME})
enable_testing()
add_test(NAME test_world_lidar COMMAND test_world_lidar)
add_test(NAME test_scene_sources COMMAND test_scene_sources)

if(catkin_FOUND)
  include_directories(${catkin_INCLUDE_DIRS})

  add_executable(world_lidar_node src/world_lidar_node.cpp)
  add_dependencies(world_lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(world_lidar_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})

  install(TARGETS world_lidar_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/topic_contract.test)
    catkin_add_gtest(${PROJECT_NAME}_core_test test/core_gtest.cpp)
    if(TARGET ${PROJECT_NAME}_core_test)
      target_link_libraries(${PROJECT_NAME}_core_test ${PROJECT_NAME})
    endif()
  endif()
endif()
