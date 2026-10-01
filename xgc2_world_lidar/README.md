# xgc2_world_lidar

Optional world-frame LiDAR / point-cloud sensor for simulated robots. It is
the shared scene sampler for lightweight sensing and explicit sampled modes on
Gazebo: one ROS-free core and a world-owned fleet node. Gazebo's native surface
ray sensor remains a separate implementation of the simple-lidar topic contract.

- **Core** (`include/xgc2_world_lidar/`, `src/world_lidar.cpp`,
  `src/scene_conversion.cpp`, `src/convex_body_conversion.cpp`): C++17 and
  Eigen only. BVH ray casting and surface sampling over closed convex solids.
- **Node** (`src/world_lidar_node.cpp`, `launch/world_lidar.launch`): a thin
  ROS1 edge. XGC2 starts one fleet node for the opted-in robot slots and any
  explicitly declared scene reference layer. Standalone per-robot launch is
  also available for sensor checks.

## Modes

| Mode | Label | A scan returns | Occlusion |
|---|---|---|---|
| `penetrating` | S0 | Every surface sample (spacing `surface_spacing`) within `[min_range, range]` and inside the angular field of view, including back faces and hidden obstacles. Optional heading / vertical-slab crop. | no |
| `raycast` | S1 | The first return of each of `h_res x v_res` beams over `h_fov_deg x v_fov_deg`. Per-beam records on `/beams`. | yes |
| `depth_frustum` | S2 | The first return of each pixel of a `width x height` pinhole camera looking along body +x. Per-pixel records on `/beams`. | yes |

Presets (`config/presets/`) load on top of `config/world_lidar.yaml`; an
explicit `mode` or numeric argument wins over the preset.

| Preset | Mode | Values |
|---|---|---|
| `bridge_equivalent` | penetrating | 8 m ball, 0.1 m spacing, buried samples kept, no crop, 20 Hz |
| `zju_cpu_crop` | penetrating | 5 m ball, heading cone `cos >= 0.5`, z slab `tan(30 deg) * 5 m` |
| `zju_cpu_crop_ego_v2` | penetrating | 5 m ball, heading half-space `cos >= 0.0`, same z slab |

## Scene sources

| `simulator` launch arg | `~scene_source` | Geometry read |
|---|---|---|
| `lightweight` | `snapshot` | `/xgc/scene/snapshot` + `/xgc/scene/state` (`xgc2_geometry_msgs/SceneSnapshot`, `SceneState`): the scene document the scene runtime serves |
| `gazebo` | `gazebo` | `/xgc2/simulation/obstacles/geometry_library` + `/xgc2/simulation/obstacles/instances` (`GeometryLibrary`, `ConvexBodyArray`): the collision geometry of every managed obstacle in the running Gazebo world, published by `xgc2_gazebo_scene` |

Both sources become the same list of convex solids. Gazebo instances map as:
`cube` → box with `scale` as full side lengths; `sphere` → radius `scale.x`;
`cylinder` → radius `scale.x`, full height `scale.z`; any other type → the
library template of that type, support points times `scale`, convex hull.
Collision meshes arrive as such templates. Models that are not managed
obstacles (ground plane, robots, unmanaged world models) are not in either
source.

## Per-robot topics

`<ns>` is the robot namespace, e.g. `/uav1`.

| Direction | Topic | Type |
|---|---|---|
| in | `pose_topic`, default `/vrpn_client_node<ns>/pose` | `geometry_msgs/PoseStamped`, world frame |
| out | `<ns>/world_lidar/points` | `PointCloud2`, frame `world`, `x y z` float32, point_step 12, stamp = pose stamp |
| out | `<ns>/world_lidar/beams` (`raycast`, `depth_frustum`) | `PointCloud2`, frame `world`, float32 `x y z dx dy dz range hit`, point_step 32 |
| service | `<ns>/world_lidar/set_enabled` | `std_srvs/SetBool`; while disabled nothing is published |

A robot that does not enable the sensor has no node and no topic.

```bash
roslaunch xgc2_world_lidar world_lidar.launch enable:=true namespace:=/uav1 \
  simulator:=lightweight mode:=raycast rate_hz:=10
```

Launch arguments: `enable` (default `false`), `namespace`, `simulator`,
`pose_topic`, `preset`, `mode`, `rate_hz`, `range`, `h_fov_deg`, `v_fov_deg`,
`h_res`, `v_res` (0 keeps the config / preset value), `publish_beams`,
`start_enabled`. Every other parameter is documented in
`config/world_lidar.yaml`.

## Tests

```bash
catkin_make run_tests_xgc2_world_lidar      # core, both scene sources, topic contract
```

- `test_world_lidar`: occlusion, penetration, surfaces and range, BVH, FOV,
  frustum, beams, heading crop, vehicle bodies, reproducibility.
- `test_scene_sources`: Gazebo instance conversion rules, and one toy scene
  given as a scene document and as Gazebo obstacle truth yields identical
  scans in every mode.
- `test/topic_contract.test`: three robots on one master (lightweight source,
  Gazebo source, sensor off) give the same topic contract and clouds for the
  first two and no node or topic for the third.

## Provenance

The core, conversion and node derive from the MIT-licensed `world_lidar_sim`
package (C++ namespace and include path renamed to `xgc2_world_lidar`).

## Shared fleet sampler

The Experiment world starts one node with a frozen `fleet_json_file` manifest
(schemaVersion 1, robots array). Each row carries its absolute namespace, mode,
range/FOV/resolution and rate. Every robot publishes world-frame XYZ at
`<namespace>/simple_lidar/points`, reading its body pose at
`/vrpn_client_node<namespace>/pose`. Simple lidar is centered on that robot;
`~sensor_pose` is refused. Real mounted sensors keep their separate models.

Geometry and surface samples are compiled once per changed scene and sampling
policy, then shared by the fleet. Penetrating queries use a spatial index before
applying per-robot FOV/crops; nonpenetrating rays use the shared geometry BVH.
Unsubscribed sensors do no scan work. Each configured rate has its own simulation
clock schedule, avoiding wall callback jitter that silently reduces scan rates.
The robots due in a tick are scanned in parallel on a fixed thread pool (one
thread per sensed robot, up to half the hardware threads; `~worker_threads`
overrides), each into its own reused cloud message. Scans are independent, so
the clouds are those of a serial pass.
A scene replacement invalidates old observations, and removed obstacles cannot
remain in the sampled map. The ROS fleet regression covers these boundaries.

The optional `referenceCloud: {"surfaceSpacing": 0.1}` manifest entry publishes
the full scene's sampled exterior at `/xgc/scene/reference_cloud`, world-frame
XYZ. It is a visualization reference, never a local sensor or an algorithm's
private map. Reference-only manifests may have an empty robots array; no robot
pose subscriptions or sensor topics are invented. Local sensors retain their
own range/FOV and sampling policy. The reference shares compiled scene data
where policies match and is published latched when consumed. Replacing,
removing, or waiting for a dynamic scene revision clears the old reference.
