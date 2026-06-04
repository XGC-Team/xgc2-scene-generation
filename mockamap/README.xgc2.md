# mockamap

ROS1 Noetic port of the `mockamap` procedural point-cloud map generator, now
owned by the XGC2 Scene Generation product.

The node publishes `sensor_msgs/PointCloud2` on `mock_map` by default. GCOPTER's
launch file remaps it to `/voxel_map`.

Build with GCOPTER from the workspace root:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="mockamap;gcopter"
```
