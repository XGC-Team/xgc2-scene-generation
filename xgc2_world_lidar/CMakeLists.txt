cmake_minimum_required(VERSION 3.16)
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
  roscpp roslib geometry_msgs nav_msgs sensor_msgs pcl_conversions xgc2_geometry_msgs
)
if(catkin_FOUND)
  catkin_install_python(PROGRAMS scripts/prepare_fleet.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  catkin_package(INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp roslib geometry_msgs nav_msgs sensor_msgs pcl_conversions xgc2_geometry_msgs
    DEPENDS PCL)
endif()
include_directories(include ${EIGEN3_INCLUDE_DIR})
add_library(${PROJECT_NAME} STATIC
  src/world_lidar.cpp src/scene_conversion.cpp src/convex_body_conversion.cpp
  src/scan_pool.cpp)
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
target_include_directories(${PROJECT_NAME}_cloud_cpu PUBLIC include ${PCL_INCLUDE_DIRS})
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
  find_package(XgcXrpc CONFIG REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)
  include_directories(${catkin_INCLUDE_DIRS})
  add_executable(world_lidar_node src/world_lidar_node.cpp src/lidar_control.cpp)
  target_compile_features(world_lidar_node PRIVATE cxx_std_20)
  add_dependencies(world_lidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(world_lidar_node PRIVATE ${PROJECT_NAME} XgcXrpc::http PkgConfig::JSONCPP ${catkin_LIBRARIES})
  install(TARGETS world_lidar_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
  add_executable(shared_cloud_cpu_node src/shared_cloud_cpu_node.cpp)
  add_dependencies(shared_cloud_cpu_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(shared_cloud_cpu_node PRIVATE ${PROJECT_NAME}_cloud_cpu ${catkin_LIBRARIES})
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
    catkin_add_gtest(${PROJECT_NAME}_core_test test/core_gtest.cpp)
    if(TARGET ${PROJECT_NAME}_core_test)
      target_link_libraries(${PROJECT_NAME}_core_test ${PROJECT_NAME})
    endif()
  endif()
endif()

# Optional adapter on the existing shared entry; no new ROS executable or alias.
include(cmake/world_lidar_gpu.cmake)
