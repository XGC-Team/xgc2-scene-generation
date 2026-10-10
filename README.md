# XGC2 Scene

Scene authoring, procedural geometry generators, and shared simulated sensing.
The canonical checkout is `products/ros1/common/scene`. This repository is not
a standalone physics simulator. Pure convex geometry algorithms belong to
`xgc2-math`.

## Source layout and responsibilities

```text
runtime/xgc2_scene_runtime/       scene documents, assembly, editing and revisions
  src/xgc2_scene_runtime/
    integrations/
      simulation_client.py       explicit simulation-v1 protocol client
      gazebo/prepare.py          Gazebo SDF/assets and world materialization
generators/
  cluttered_environment/         convex obstacle scenarios and legacy ROS publisher
  mockamap/                      procedural point-cloud maps
sensors/xgc2_world_lidar/         CPU/GPU sensing library and ROS entries
```

Shared ROS messages are maintained in `ros1/common/ros1-msgs`. ROS package,
Python domain, native CMake target and Debian package names remain stable.
The integration Python imports are explicitly namespaced as
`xgc2_scene_runtime.integrations.simulation_client` and
`xgc2_scene_runtime.integrations.gazebo.prepare`; old import locations are removed.

The product ID `xgc2-scene-generation` is retained for the existing release
dependency graph; it does not describe a physics engine.

| Component | Responsibility | Actual consumers |
| --- | --- | --- |
| `xgc2_scene_runtime` | Scene document validation, placement/assembly, edits, persistence, revisions and obstacle playback; ROS publication and XRPC authoring service | Core scene assembly (`python3 -m xgc2_scene_runtime.assembly`), GCS scene workflow, Gazebo scene application, lightweight scene sensing, UGV reset scene consumers |
| `cluttered_environment` | Configurable convex obstacle scenarios and their legacy ROS scenario publisher | Scenario launches and obstacle-rich experiments; not the owner of simulator physics |
| `mockamap` | Procedural point-cloud maps | GCOPTER and EGO planner demo launches |
| `xgc2_world_lidar` | ROS-free CPU/GPU sensing implementations plus ROS fleet and per-robot entries | Native XSIM builds its sensor library directly from this owner; ROS fleet sensing consumes scene snapshots or Gazebo obstacle truth |
| `integrations.simulation_client` / `integrations.gazebo.prepare` | Explicit simulation-v1 client; Gazebo SDF/assets preparation is separately namespaced | Gazebo world preparation and scene apply/motion integration |

`xgc2_geometry_msgs` is maintained in
[`XGC-Team/xgc2-ros-msgs`](https://github.com/XGC-Team/xgc2-ros-msgs/tree/noetic/xgc2_geometry_msgs).
Its eleven message definitions, ROS package name/version, message MD5s and MIT
license declaration are unchanged from this repository's
[`869cc7824d021bd21b3b23aceb266e4e71eeb9d7`](https://github.com/XGC-Team/xgc2-scene-generation/tree/869cc7824d021bd21b3b23aceb266e4e71eeb9d7/xgc2_geometry_msgs).
Install `ros-noetic-xgc2-geometry-msgs` separately; this repository consumes it
and no longer builds or packages a private copy. The Melodic message package is
also owned and verified by the shared message repository. Runtime/generator/
sensor packages here retain their existing Noetic/Focal scope.

## Boundary and remaining separation

The scene domain owns authored geometry and versioned scene intent. A simulator
owns physics, robot state and the actual application of that intent; a consumer
acknowledgement does not make the scene service a universal simulator controller.
A native simulator world can retain its own geometry authority. Therefore the
whole repository cannot be treated as one map server already controlling every
simulator. `xgc2_scene_runtime` is the shared scene service within this collection.

The source groups make these responsibilities explicit within the existing
product. Generators produce scene geometry or point clouds; they do not own
authoring persistence or physics. Sensing consumes geometry and robot poses to
produce observations; it does not edit scene assets. Gazebo preparation
interprets SDF/assets and remains separate from the scene document model.
`ros_node.py` still wires the simulator integration when explicitly requested.
Moving the checkout does not remove that dependency or establish a universal
simulator implementation. Generator output, authored scene geometry and
simulated sensor observations remain distinct products.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-scene-generation
```

The aggregate installs the four implementation packages and their geometry
message dependency. Each implementation remains independently packaged; the
shared message dependency has its own version and is not pinned to this
repository's Debian revision.

## Smoke Test

```bash
rospack find xgc2_scene_runtime
rospack find cluttered_environment
rospack find mockamap
rospack find xgc2_world_lidar
```
