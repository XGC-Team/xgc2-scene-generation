cmake_minimum_required(VERSION 3.16)
project(native_lidar_control_fixture LANGUAGES CXX)
find_package(XgcXrpc CONFIG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JsonCpp REQUIRED IMPORTED_TARGET jsoncpp)
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(Eigen3 REQUIRED)
set(lidar_dir "${CMAKE_CURRENT_SOURCE_DIR}/../..")
add_executable(lidar_control_fixture main.cpp "${lidar_dir}/src/lidar_control.cpp")
target_compile_features(lidar_control_fixture PRIVATE cxx_std_20)
target_compile_options(lidar_control_fixture PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(lidar_control_fixture PRIVATE "${lidar_dir}/include")
target_include_directories(lidar_control_fixture SYSTEM PRIVATE /opt/ros/noetic/include)
target_link_libraries(lidar_control_fixture PRIVATE XgcXrpc::http PkgConfig::JsonCpp Boost::thread Boost::system
  /opt/ros/noetic/lib/libroscpp.so /opt/ros/noetic/lib/libroscpp_serialization.so
  /opt/ros/noetic/lib/librostime.so /opt/ros/noetic/lib/librosconsole.so)

set(GEOMETRY_MESSAGES_INCLUDE "" CACHE PATH "Generated product geometry messages")
if(GEOMETRY_MESSAGES_INCLUDE)
  add_executable(world_lidar_native_fixture
    "${lidar_dir}/src/world_lidar_node.cpp" "${lidar_dir}/src/lidar_control.cpp"
    "${lidar_dir}/src/world_lidar.cpp" "${lidar_dir}/src/scene_conversion.cpp"
    "${lidar_dir}/src/convex_body_conversion.cpp" "${lidar_dir}/src/scan_pool.cpp")
  target_compile_features(world_lidar_native_fixture PRIVATE cxx_std_20)
  target_include_directories(world_lidar_native_fixture PRIVATE "${lidar_dir}/include")
  target_include_directories(world_lidar_native_fixture SYSTEM PRIVATE /opt/ros/noetic/include "${GEOMETRY_MESSAGES_INCLUDE}")
  target_link_libraries(world_lidar_native_fixture PRIVATE XgcXrpc::http PkgConfig::JsonCpp Eigen3::Eigen Boost::thread Boost::system
    /opt/ros/noetic/lib/libroscpp.so /opt/ros/noetic/lib/libroscpp_serialization.so
    /opt/ros/noetic/lib/librostime.so /opt/ros/noetic/lib/librosconsole.so /opt/ros/noetic/lib/libroslib.so)
endif()
