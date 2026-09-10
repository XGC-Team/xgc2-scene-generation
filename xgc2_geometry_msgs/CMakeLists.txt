cmake_minimum_required(VERSION 3.0.2)
project(xgc2_geometry_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  GeometryTemplate.msg
  GeometryLibrary.msg
  ConvexBodyInstance.msg
  ConvexBodyArray.msg
  SceneGeometry.msg
  ScenePart.msg
  SceneObstacle.msg
  SceneSnapshot.msg
  SceneObstacleState.msg
  SceneState.msg
  SceneConsumerStatus.msg
)

add_service_files(FILES ApplyScene.srv SceneCommand.srv)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)
