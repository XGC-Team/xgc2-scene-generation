# Geometry Split Summary

## Conclusion

The geometry stack is split by semantic responsibility:

- `libxgc2-geometry-dev`: pure C++ convex sets, convex hull helpers, and GJK queries.
- `xgc2_geometry_msgs`: ROS message definitions for geometry templates and obstacle instances.
- `cluttered_environment`: ROS scene loading, marker publishing, TF publishing, and obstacle-instance publishing.

`cluttered_environment` depends on the C++ library and the message package. It does not own generic geometry data structures.

## Public ROS Topics

- `/xgc2_geometry/geometry_library`
- `/xgc2_geometry/static_body_instances`
- `/xgc2_geometry/dynamic_body_instances`

## Current Structure

```text
products/common/geometry/
├── include/xgc2_geometry/
│   ├── collision/
│   ├── geometry/
│   └── occupied_sets/
└── CMakeLists.txt

xgc2_geometry_msgs/
├── msg/
└── CMakeLists.txt

cluttered_environment/
├── include/cluttered_environment/
├── src/
├── config/
├── launch/
└── docs/
```

## Build Verification

```bash
catkin_make --pkg xgc2_geometry_msgs cluttered_environment
```
