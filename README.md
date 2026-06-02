# XGC2 Convex Geometry

ROS1 convex geometry messages, collision geometry helpers, and simulation environment tools for XGC2.

This repository contains:

- `convex_geometry`: convex body messages, geometry templates, and header-only collision helpers.
- `convex_geometry_environment`: scenario manager and configuration assets for static and dynamic convex obstacles.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-convex-geometry-environment
```

## Smoke Test

```bash
rospack find convex_geometry
rospack find convex_geometry_environment
```
