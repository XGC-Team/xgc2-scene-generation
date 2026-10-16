# World LiDAR numerical library

This owner retains the ROS-free CPU geometry conversion, surface sampling,
ray queries, scan pool, numerical tests and benchmark. The standalone
`world_lidar_node`, fleet preparer and their ROS launch/control/tests are
retired. Live Scene sensing belongs to the C++ Scene Adapter's own LiDAR edge;
there is no fallback publisher or second scene authority here.

`src/world_lidar.cpp`, `src/scene_conversion.cpp` and `src/scan_pool.cpp`
build `xgc2_world_lidar`. `library/CMakeLists.txt` retains the standalone
`XgcWorldLidar::Geometry` export used by source consumers. The normal catkin
package installs the numerical archive and headers. None of these libraries
starts ROS or owns experiment lifecycle.

The optional `xgc2_world_lidar_cloud_gpu` library adapter consumes the shared
`xgc2_sensing` API and explicit sensor metadata. It does not add a ROS node.
The original renderer/observation model and corresponding licenses remain in
their source owners; retiring a ROS executable changes no kernel or scientific
observation formula.

## Numerical checks

`test_world_lidar`, `test_scene_sources`, `test_observation_contract` and
`test_ring_geometry` cover the retained geometry/sampling implementations.
`bench_world_lidar` remains an explicit numerical benchmark.
The historical comparison source and fixed reference build inputs remain;
the removed PR6 runner no longer launches an obsolete ROS executable.

Native ROS demand, typed snapshot/state, pose stamps, body roster, application
invalidation and source control are owned by Adapter's `edge_test` and
`transport_test`; their existence is not a claim that they were rerun here.
Full experiment, GPU hardware and desktop acceptance remain separate from
library correctness or package assembly.
