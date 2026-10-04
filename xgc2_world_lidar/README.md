# xgc2_world_lidar

## Shared static-cloud entry

`shared_cloud_cpu_node` is the existing shared entry; its target and ROS name
remain unchanged. It consumes one initial `sensor_msgs/PointCloud2` world cloud,
then unsubscribes. The same pose subscriptions, timer, roster output publishers
and ROS shutdown serve the selected backend. There is no `set_enabled` service
in this entry. `world_lidar_node` remains the separate existing Scene-geometry
and ScanPool entry; its services are not imported here.

Select `observation_model` and `backend` explicitly:

| Model / backend | Observation |
| --- | --- |
| `crop_through` / `cpu` | Original PCL voxel/radius/full-quaternion body-X and optional world-Z crop; no occlusion. Caller values, rate, frame and zero/pose stamp policy stay unchanged. |
| `lidar_scan` / `gpu` | Original MARSIM spherical angular projection with nearest projected/splatted depth. One renderer/context on the ROS owner thread uploads the static world once and scans all roster poses sequentially. |

The GPU model is not a CPU-through equivalent or a pinhole camera. It reads
`range_m`, `min_range_m`, `h_fov_deg`, `v_fov_deg`, `h_res`, `v_res`,
`point_cover_spacing_m` and `publish_rate_hz`. The original kernel requires
equal native-float angular steps (`h_fov_deg/h_res == v_fov_deg/v_res`). The
point-cover/near ratio must be within the original `asin` domain. Near/far,
spacing and rate must be finite and positive; far must exceed near. Original
readback still keeps **`depth > near && depth < far - 0.1`**, rather than an
inclusive range. `point_cover_spacing_m` controls the original GPU point cover,
not CPU prevoxel. Supplied GPU `prevoxel_leaf_m` is explicitly rejected.

Both backends use `input_cloud_topic`, `pose_type`, `pose_topics`, `output_topics`,
`frame_id` and `stamp_policy`. Poses must already be in the input cloud's world
coordinates; this entry performs no TF/extrinsic conversion. The GPU output
retains original XYZ/intensity fields, with a legitimate empty scan allowed.
Unknown model/backend combinations and a GPU request in a CPU-only build fail
without fallback. Pinhole/through GPU, moving-body sensing, dynamic world
replacement and pose-loss invalidation are not implemented by this entry.
Legacy camera/noise/pattern declarations are not mapped to GPU sensor behavior.

CPU compilation is the default. To include the original GPU adapter, enable
`XGC_WORLD_LIDAR_GPU=ON`; optional CMake then finds GLFW, GLM, OpenGL/Khronos,
OpenCV, PCL and OpenMP. These dependencies are external: GPU dependencies are
not added to the default CPU package closure. With the ordinary ROS and scene
message underlays sourced, a normal source checkout uses the bundled
`vendored/MARSIM` by default; `XGC_WORLD_LIDAR_GPU_SOURCE_ROOT` remains an
explicit override. No `/tmp` source path is required.

```bash
prefix=/var/lib/xgc2-local-swarm/user-project-build/ws_world_lidar/install
cmake -S . -B build -DCATKIN_ENABLE_TESTING=OFF \
  -DXGC_WORLD_LIDAR_GPU=ON -DCMAKE_INSTALL_PREFIX="$prefix"
cmake --build build --target shared_cloud_cpu_node -- -j1
DESTDIR=/private/stage cmake --install build --component cloud_cpu
DESTDIR=/private/stage cmake --install build --component cloud_gpu
```

`cloud_cpu` retains the existing node/library/header install; `cloud_gpu`
adds the original shader bytes, license, corresponding fixed source, package
metadata and example GPU parameters. The kernel's compiled `ROOT_DIR` embeds
the ordinary install prefix above. `DESTDIR` stages that prefix; it does not
change the embedded path. Consume the staged tree at the same prefix, or build
for the intended ordinary prefix. The absolute executable remains
`$prefix/lib/xgc2_world_lidar/shared_cloud_cpu_node`; no target alias is added.
The example GPU YAML deliberately leaves world/pose/output bindings unbound.
Normal source builds and staged installation do not establish installed ROS,
hardware or geometric correctness; those require their separately authorized
ordinary consumption.

`vendored/MARSIM` retains the fixed ubuntu20 commit
`2a287bb196eb35375636c3aa6ac6c6be45ebb1f3`, original GPL-2.0 license,
upstream header and approved memory-entry patch. Its original renderer and
shader bytes are retained. The optional GPU-linked executable has GPL-2.0
obligations; the existing MIT CPU/Scene source remains separately licensed.
No maps, results, GLM/GLFW copies, dynamics or new renderer are vendored.

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
- `test/cache_lifecycle.test`: disabled reference invalidation and reenable,
  unsubscribed seeded scans, serial/pool byte equality, exact pose stamps,
  optional neighbor returns, self exclusion and unsensed neighbor bodies.

With the local PR6 base object and the installed Noetic build image, run
`xgc2_world_lidar/test/run_pr6_validation.sh [evidence-directory]` from the
repository. This uses read-only source mounts in offline containers (2 CPUs
for compilation, 1 CPU for tests), keeps logs and XML results, compares the
original and incremental cores byte for byte across all modes and scene edits,
compares real original/candidate ROS nodes on dynamic scene edits, and runs
the lifecycle tests with the installed executable. It does not start a station
or fetch dependencies. These are sensor and installation checks, not a complete
planning/control experiment or a Gazebo surface-sensor acceptance run.

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
policy, then shared by the fleet. A changed revision (moving obstacles) is built
from the installed one: only obstacles that changed, and without buried samples
the obstacles they touch, are converted, compiled, sampled and indexed again;
the result equals a build from scratch. Penetrating queries use a spatial index before
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

## World-model body roster (raycast P1)

The existing fleet manifest schema version 1 accepts a separate `bodies` array.
`robots` remains only the sensor roster. A body is an explicit sphere envelope
centered at its world-model truth pose (not a certified robot mesh):

```json
{"schemaVersion":1,"robots":[{"namespace":"/A","bodyId":41,"mode":"raycast"}],
 "bodies":[{"id":41,"namespace":"/A","poseTopic":"/xgc/simulation/body/A/pose",
            "geometry":{"type":"sphere","radiusMeters":0.3}},
           {"id":7,"namespace":"/B","poseTopic":"/xgc/simulation/body/B/pose",
            "geometry":{"type":"sphere","radiusMeters":0.3}}]}
```

Core provides every simulated world member, including B without a sensor, with
stable body IDs, explicit sphere geometry, and world-frame PoseStamped truth
from the same model that produces motion. The sphere and the pose use the same
origin; radius is supplied from that model's geometry boundary, never guessed
by this node. Unsupported geometry is rejected. No second world is created.

Raycast/depth sensors include that roster and exclude their own bodyId. Walls
still select the nearest first return. Their default scan origin and exact
output stamp come from their own body truth topic; an explicit sensor poseTopic
remains available in the manifest. Body-free penetrating/crop keeps its existing
VRPN source, three XYZ fields and observation baseline; `vehicle_bodies` remains
its explicit opt-in. Legacy manifests without bodies retain their behavior.

The body roster is frozen for the run/world instance. Algorithm/provider stop
cannot remove a model or its body: model-owned truth publication must continue.
A missing, invalid, future or stale truth sample fails closed for body-aware
scans, rather than turning that member into observed-free space. The roster is
replaced only with the owning world model lifecycle, never with sensor demand.
`test/body_roster.test` exercises unsensed-body hits, wall occlusion, self IDs,
provider stop, interrupted/resumed truth and unchanged penetrating baseline.
