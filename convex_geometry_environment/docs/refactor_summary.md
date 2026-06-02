# Convex Geometry Environment And Convex Geometry Refactor Summary

## Conclusion

`convex_geometry_environment` is now a scene package, not the owner of generic convex geometry data structures. Shared convex body descriptions, occupied sets, convex geometry helpers, and GJK separation queries live in the dedicated `convex_geometry` package.

This keeps scene loading and RViz obstacle visualization in `convex_geometry_environment`, while `formation_generator` and future UAV/UGV planners consume the same geometry package directly.

## Main Results

1. Package responsibilities are separated.
   - `convex_geometry_environment`: scene configuration, obstacle runtime objects, marker publication, and body-instance publication.
   - `convex_geometry`: reusable convex body messages, occupied-set abstractions, geometry helpers, and collision/separation query algorithms.
   - `formation_generator`: DMPC scheduling and optimization logic that consumes `convex_geometry`.

2. Convex geometry data structures are centralized in `convex_geometry`.
   - `convex_geometry/occupied_sets/`
   - `convex_geometry/geometry/`
   - `convex_geometry/collision/`
   - `convex_geometry/msg/GeometryTemplate.msg`
   - `convex_geometry/msg/GeometryLibrary.msg`
   - `convex_geometry/msg/ConvexBodyInstance.msg`
   - `convex_geometry/msg/ConvexBodyArray.msg`

3. The scene obstacle runtime remains local to `convex_geometry_environment`.
   - `scene/obstacles/obstacle_base.*`
   - `scene/obstacles/obstacle_factory.*`
   - `scene/obstacles/primitive_obstacles.*`
   - `scene/obstacles/polytope_obstacles.*`

   These classes load scenario semantics and convert them into reusable `convex_geometry` messages.

4. The external geometry interface is unified.
   - Shared geometry templates are published on:
     - `/convex_geometry/geometry_library`
   - Static body instances are published on:
     - `/convex_geometry/static_body_instances`
   - Dynamic body instances are published on:
     - `/convex_geometry/dynamic_body_instances`

5. Obsolete duplicate ownership was removed.
   - `convex_geometry_environment` no longer defines generic occupied-set headers or geometry helper headers.
   - `convex_geometry_environment` no longer owns geometry/body message definitions.
   - `formation_generator` no longer owns GJK collision query headers.
   - Legacy obstacle-instance message definitions were removed in favor of `ConvexBodyInstance`/`ConvexBodyArray`.

## Current Structure

```text
convex_geometry/
├── include/convex_geometry/
│   ├── collision/
│   ├── geometry/
│   └── occupied_sets/
├── msg/
└── CMakeLists.txt

convex_geometry_environment/
├── include/convex_geometry_environment/
│   ├── scene/obstacles/
│   └── scenario_manager.h
├── src/
│   ├── scene/obstacles/
│   ├── scenario_manager.cpp
│   └── scenario_node.cpp
├── config/
├── launch/
└── docs/
```

## Build Verification

The current unified geometry split should be verified with:

```bash
catkin_make --pkg convex_geometry
catkin_make --pkg convex_geometry_environment
catkin_make --pkg formation_generator
```
