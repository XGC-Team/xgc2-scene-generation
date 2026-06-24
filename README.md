# XGC2 Scene Generation

ROS1 scene-generation tools for obstacle-rich simulation and planner demos.

This repository contains:

- `xgc2_geometry_msgs`: convex body template and obstacle instance messages.
- `cluttered_environment`: scenario manager and configuration assets for static and dynamic convex obstacles.
- `mockamap`: procedural point-cloud map generator for voxel-map based planner demos.
- `libxgc2-math-dev`: pure C++ convex-set, convex-hull, and GJK implementation installed from the XGC2 common apt package.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-scene-generation
```

## Smoke Test

```bash
rospack find xgc2_geometry_msgs
rospack find cluttered_environment
rospack find mockamap
```
