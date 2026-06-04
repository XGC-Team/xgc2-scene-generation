cmake_minimum_required(VERSION 3.0.2)
project(cluttered_environment)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  xgc2_geometry_msgs
)

## Find system libraries
find_package(Eigen3 REQUIRED)
find_package(xgc2_geometry REQUIRED CONFIG)

## Catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    xgc2_geometry_msgs
  DEPENDS
    EIGEN3
)

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare the scenario node executable
add_executable(${PROJECT_NAME}_node
  src/scenario_node.cpp
  src/scenario_manager.cpp
  src/scene/obstacles/obstacle_base.cpp
  src/scene/obstacles/primitive_obstacles.cpp
  src/scene/obstacles/polytope_obstacles.cpp
  src/scene/obstacles/obstacle_factory.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  xgc2_geometry::geometry
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_smoke_test
    test/cluttered_environment_smoke_test.cpp
    src/scene/obstacles/obstacle_base.cpp
    src/scene/obstacles/primitive_obstacles.cpp
    src/scene/obstacles/polytope_obstacles.cpp
    src/scene/obstacles/obstacle_factory.cpp
  )

  if(TARGET ${PROJECT_NAME}_smoke_test)
    target_link_libraries(${PROJECT_NAME}_smoke_test
      ${catkin_LIBRARIES}
      ${EIGEN3_LIBRARIES}
      xgc2_geometry::geometry
    )
  endif()
endif()

## Install targets
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY docs/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/velocity_commander.py
  scripts/yaw_to_quaternion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
