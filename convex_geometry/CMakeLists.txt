cmake_minimum_required(VERSION 3.0.2)
project(convex_geometry)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  GeometryTemplate.msg
  GeometryLibrary.msg
  ConvexBodyInstance.msg
  ConvexBodyArray.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(convex_geometry_unit_test
    test/convex_geometry_unit_test.cpp
  )
  add_dependencies(convex_geometry_unit_test
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(convex_geometry_unit_test
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(convex_geometry_smoke_test
    test/convex_geometry_smoke_test.cpp
  )
  add_dependencies(convex_geometry_smoke_test
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(convex_geometry_smoke_test
    ${catkin_LIBRARIES}
  )
endif()
