cmake_minimum_required(VERSION 3.0.2)
project(xgc2_world_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search kdtree)
find_package(catkin QUIET COMPONENTS
  roscpp roslib geometry_msgs nav_msgs sensor_msgs std_srvs pcl_conversions
)
if(catkin_FOUND)
  find_package(xgc2_geometry_msgs QUIET)
  set(_scene_msg_dependency)
  if(xgc2_geometry_msgs_FOUND)
    set(_scene_msg_dependency xgc2_geometry_msgs)
  endif()
  catkin_package(INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_cloud_cpu ${PROJECT_NAME}_sensor_system
    CATKIN_DEPENDS roscpp roslib geometry_msgs nav_msgs sensor_msgs std_srvs pcl_conversions
    DEPENDS PCL)
endif()
include_directories(include ${EIGEN3_INCLUDE_DIR})
add_library(${PROJECT_NAME} STATIC
  src/world_lidar.cpp src/scene_conversion.cpp src/convex_body_conversion.cpp
  src/scan_pool.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME Core)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)
if(TARGET Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
else()
  target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()
# Shared point-cloud CPU crop uses this world owner's point source and one index.
# Other observation models keep their existing implementation and sampling policy.
add_library(${PROJECT_NAME}_cloud_cpu STATIC
  src/sensor_metadata.cpp src/shared_cloud_cpu.cpp)
set_target_properties(${PROJECT_NAME}_cloud_cpu PROPERTIES POSITION_INDEPENDENT_CODE ON EXPORT_NAME CloudCpu)
target_include_directories(${PROJECT_NAME}_cloud_cpu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include> ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_cloud_cpu PUBLIC ${PCL_LIBRARIES})
if(TARGET Eigen3::Eigen)
  target_link_libraries(${PROJECT_NAME}_cloud_cpu PUBLIC Eigen3::Eigen)
else()
  target_include_directories(${PROJECT_NAME}_cloud_cpu PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()

enable_testing()
foreach(suite world_lidar scene_sources observation_contract)
  add_executable(test_${suite} test/test_${suite}.cpp)
  target_link_libraries(test_${suite} PRIVATE ${PROJECT_NAME})
  add_test(NAME test_${suite} COMMAND test_${suite})
endforeach()
# Fleet scan benchmark (not a test): see the usage at the top of the source.
add_executable(bench_world_lidar test/bench_world_lidar.cpp)
target_link_libraries(bench_world_lidar PRIVATE ${PROJECT_NAME})

if(catkin_FOUND)
  include_directories(${catkin_INCLUDE_DIRS})
  add_library(${PROJECT_NAME}_sensor_system STATIC src/world_sensor_system.cpp)
  set_target_properties(${PROJECT_NAME}_sensor_system PROPERTIES
    POSITION_INDEPENDENT_CODE ON EXPORT_NAME Sensors)
  target_include_directories(${PROJECT_NAME}_sensor_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
    ${catkin_INCLUDE_DIRS})
  target_compile_features(${PROJECT_NAME}_sensor_system PUBLIC cxx_std_17)
  target_link_libraries(${PROJECT_NAME}_sensor_system PUBLIC
    ${PROJECT_NAME} ${PROJECT_NAME}_cloud_cpu ${catkin_LIBRARIES})
  add_dependencies(${PROJECT_NAME}_sensor_system ${catkin_EXPORTED_TARGETS})
  install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_cloud_cpu ${PROJECT_NAME}_sensor_system
    EXPORT XgcWorldLidarTargets ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} COMPONENT world_sensor_system)
  include(CMakePackageConfigHelpers)
  configure_package_config_file(cmake/XgcWorldLidarConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/XgcWorldLidarConfig.cmake
    INSTALL_DESTINATION share/cmake/XgcWorldLidar)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/XgcWorldLidarConfig.cmake
    DESTINATION share/cmake/XgcWorldLidar COMPONENT world_sensor_system)
  install(EXPORT XgcWorldLidarTargets NAMESPACE XgcWorldLidar::
    DESTINATION share/cmake/XgcWorldLidar COMPONENT world_sensor_system)
  # The kernel/execution owner does not consume the Scene ROS DTO. Only this
  # legitimate non-pureSim edge needs the installed common Scene interface.
  if(xgc2_geometry_msgs_FOUND)
    add_executable(world_lidar_node src/world_lidar_node.cpp)
    target_include_directories(world_lidar_node PRIVATE ${xgc2_geometry_msgs_INCLUDE_DIRS})
    add_dependencies(world_lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${xgc2_geometry_msgs_EXPORTED_TARGETS})
    target_link_libraries(world_lidar_node PRIVATE ${PROJECT_NAME} ${catkin_LIBRARIES})
    install(TARGETS world_lidar_node
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  endif()
  install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} COMPONENT world_sensor_system)
  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
  add_executable(shared_cloud_cpu_node src/shared_cloud_cpu_node.cpp)
  add_dependencies(shared_cloud_cpu_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(shared_cloud_cpu_node PRIVATE ${PROJECT_NAME}_sensor_system ${catkin_LIBRARIES})
  install(TARGETS shared_cloud_cpu_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} COMPONENT cloud_cpu)
  install(TARGETS ${PROJECT_NAME}_cloud_cpu
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} COMPONENT cloud_cpu)
  install(FILES include/xgc2_world_lidar/sensor_metadata.hpp
    include/xgc2_world_lidar/shared_cloud_cpu.hpp
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} COMPONENT cloud_cpu)
  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/topic_contract.test)
    add_rostest(test/observation_contract.test)
    add_rostest(test/fleet_contract.test)
    add_rostest(test/cache_lifecycle.test)
    add_rostest(test/body_roster.test)
    catkin_add_gtest(${PROJECT_NAME}_core_test test/core_gtest.cpp test/world_sensor_system_test.cpp)
    if(TARGET ${PROJECT_NAME}_core_test)
      target_link_libraries(${PROJECT_NAME}_core_test ${PROJECT_NAME} ${PROJECT_NAME}_sensor_system)
    endif()
  endif()
endif()

# Optional adapter on the existing shared entry; no new ROS executable or alias.
include(cmake/world_lidar_gpu.cmake)
