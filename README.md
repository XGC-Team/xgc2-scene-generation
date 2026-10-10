# XGC2 Scene Generation

Scene authoring, procedural geometry generators, and shared simulated sensing.
This repository is not a standalone physics simulator. The local checkout name
`convex_geometry` is historical; pure convex geometry algorithms belong to
`xgc2-math`.

## Current responsibilities and consumers

| Component | Responsibility | Actual consumers |
| --- | --- | --- |
| `xgc2_scene_runtime` | Scene document validation, placement/assembly, edits, persistence, revisions and obstacle playback; ROS publication and XRPC authoring service | Core scene assembly (`python3 -m xgc2_scene_runtime.assembly`), GCS scene workflow, Gazebo scene application, lightweight scene sensing, UGV reset scene consumers |
| `cluttered_environment` | Configurable convex obstacle scenarios and their legacy ROS scenario publisher | Scenario launches and obstacle-rich experiments; not the owner of simulator physics |
| `mockamap` | Procedural point-cloud maps | GCOPTER and EGO planner demo launches |
| `xgc2_world_lidar` | ROS-free CPU/GPU sensing implementations plus ROS fleet and per-robot entries | Native XSIM builds its sensor library directly from this owner; ROS fleet sensing consumes scene snapshots or Gazebo obstacle truth |
| `xgc2_scene_runtime.prepare` / `simulation_client` | Currently colocated Gazebo asset preparation and simulation extension calls | Gazebo world preparation and scene apply/motion integration |

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

Further separation should keep the scene domain together, expose procedural
generators as scene/map producers, put reusable sensing under a sensor owner,
and place Gazebo-specific preparation in its integration owner. These are
responsibility boundaries, not completed source moves. In particular,
`prepare.py` interprets Gazebo SDF/assets and `ros_node.py` still has explicit
Gazebo integration. Moving this whole tree to `common/` would retain those
couplings. Generator output, authored scene geometry and simulated sensor
observations are distinct products.

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
