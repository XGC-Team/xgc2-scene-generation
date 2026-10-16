# XGC2 Scene

This product contains pure Python scene-authoring/reference tools, procedural
map generators, and numerical sensing libraries. The canonical checkout is
`products/ros1/common/scene`. It is not a standalone physics simulator.

## Owners and consumers

| Component | Responsibility | Current consumption |
| --- | --- | --- |
| `runtime/scene_runtime` | Pure document validation, assembly, generation, motion, store/undo/save and golden references | Finite authoring tools and the native adapter's offline differential corpus |
| `generators/cluttered_environment` | Procedural obstacle generation | Explicit authoring/generator callers |
| `generators/mockamap` | Procedural point-cloud generation | Explicit selected point-cloud inputs |
| `sensors/world_lidar` | ROS-free geometry conversion, scan pool and numerical sensing; optional GPU library adapter | Source library consumers and numerical/reference tests |

The live Scene owner is the C++ `xgc_scene_ros1_adapter` in the ROS1 Adapter
product. Core's managed Scene workflow prepares one selected source and starts
that owner. Its compiled definition is projected to the shared geometry table;
state updates carry dynamic poses. Adapter-owned LiDAR edges consume that same
definition. Gazebo and XSim retain their separately declared simulator roles.

The old Python `scene_node`/`scene_prepare.py` executables, their ROS launch,
and the separate `world_lidar_node` executable/launch are retired. They are not
alternate Scene publishers or sensing owners. Caller-supplied paper platform
plans must explicitly provide exactly one Scene owner rather than create one
implicitly. Historical reference code and corpus provenance do not authorize
starting an old runtime.

Shared ROS messages belong to `ros1/common/ros1-msgs`; pure geometry and the
binary geometry codec belong to `common/math`. This repository does not package
private message definitions or maintain another geometry wire format.

The `xgc2-scene-generation` product and its four existing package identities
remain for the release dependency graph. `xgc2_scene_runtime` now installs pure
Python/reference tools; `xgc2_world_lidar` installs its numerical library and
headers. The native adapter is a separate product dependency of its callers.

## Install and checks

```bash
sudo apt install ros-noetic-xgc2-scene-generation
python3 -c 'from xgc2_scene_runtime.store import SceneStore'
```

Pure Python store/generation/motion tests remain with their source. Native
Scene projection, playback, application receipts, typed ROS messages and
LiDAR edge lifecycle are tested by the owning C++ Adapter suites. Neither a
package build nor an offline library test establishes live desktop or flight
acceptance.
