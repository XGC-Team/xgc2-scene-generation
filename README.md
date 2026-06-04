# XGC2 Cluttered Environment

ROS1 geometry messages and simulation environment tools backed by the pure C++ XGC2 geometry library.

This repository contains:

- `xgc2_geometry_msgs`: convex body template and obstacle instance messages.
- `cluttered_environment`: scenario manager and configuration assets for static and dynamic convex obstacles.
- `libxgc2-geometry-dev`: pure C++ convex-set, convex-hull, and GJK implementation installed from the XGC2 common apt package.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-cluttered-environment
```

## Smoke Test

```bash
rospack find xgc2_geometry_msgs
rospack find cluttered_environment
```
