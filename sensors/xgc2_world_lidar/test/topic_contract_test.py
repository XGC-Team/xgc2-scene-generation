#!/usr/bin/env python3
"""Both scene sources give the same per-robot topic contract and clouds; a
robot that leaves the sensor off has no node and no topic."""
import math
import struct
import unittest

import rospy
import rosgraph
import rostest
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2
from xgc2_geometry_msgs.msg import (ConvexBodyArray, ConvexBodyInstance, GeometryLibrary, GeometryTemplate,
                                    SceneGeometry, SceneObstacle, ScenePart, SceneSnapshot)


def yaw_quaternion(yaw):
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


# (id, type, x, y, z, yaw, size or (radius, height))
TOY = [
    ("wall", "box", 4.0, 0.0, 1.0, 0.3, (0.4, 6.0, 2.0)),
    ("pillar", "cylinder", 0.0, 4.0, 1.5, 0.0, (0.5, 3.0)),
    ("ball", "sphere", -3.0, -3.0, 1.0, 0.0, (0.8, 0.0)),
]


def snapshot():
    msg = SceneSnapshot(scene_id="toy", epoch="e1", revision=1)
    msg.header.frame_id = "world"
    for ident, kind, x, y, z, yaw, dims in TOY:
        geometry = SceneGeometry(type=kind)
        if kind == "box":
            geometry.size = Vector3(*dims)
        else:
            geometry.radius, geometry.height = dims
        part = ScenePart(id="p0", pose=Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)), geometry=geometry)
        msg.obstacles.append(SceneObstacle(id=ident, name=ident, dynamic=False, motion_type="static",
                                           pose=Pose(Point(x, y, z), yaw_quaternion(yaw)), parts=[part]))
    return msg


def gazebo_truth():
    library = GeometryLibrary(templates=[GeometryTemplate(type="cube", resolution=0, support_points=[
        Point(sx * 0.5, sy * 0.5, sz * 0.5) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])])
    library.header.frame_id = "world"
    bodies = ConvexBodyArray()
    bodies.header.frame_id = "world"
    for index, (ident, kind, x, y, z, yaw, dims) in enumerate(TOY):
        scale = Vector3(*dims) if kind == "box" else Vector3(dims[0], dims[0], dims[1] if kind == "cylinder" else dims[0])
        bodies.instances.append(ConvexBodyInstance(
            id=index + 1, name=ident, geometry_type={"box": "cube"}.get(kind, kind),
            pose=Pose(Point(x, y, z), yaw_quaternion(yaw)), scale=scale, is_static=True))
    return library, bodies


def xyz(cloud):
    step = cloud.point_step
    return [struct.unpack_from("<fff", cloud.data, i * step) for i in range(cloud.width * cloud.height)]


class TopicContract(unittest.TestCase):
    def setUp(self):
        latched = dict(queue_size=1, latch=True)
        self.snapshot_pub = rospy.Publisher("/xgc/scene/snapshot", SceneSnapshot, **latched)
        self.library_pub = rospy.Publisher("/xgc2/simulation/obstacles/geometry_library", GeometryLibrary, **latched)
        self.instances_pub = rospy.Publisher("/xgc2/simulation/obstacles/instances", ConvexBodyArray, **latched)
        self.pose_pubs = [rospy.Publisher("/vrpn_client_node/%s/pose" % ns, PoseStamped, queue_size=1)
                          for ns in ("uav1", "ugv2", "ugv3", "uav4")]
        self.snapshot_pub.publish(snapshot())
        library, bodies = gazebo_truth()
        self.library_pub.publish(library)
        self.instances_pub.publish(bodies)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_poses)

    def tearDown(self):
        self.timer.shutdown()

    def publish_poses(self, _):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "world"
        pose.pose = Pose(Point(0.5, 0.2, 1.0), yaw_quaternion(0.4))
        for pub in self.pose_pubs:
            pub.publish(pose)

    def wait_cloud(self, topic):
        return rospy.wait_for_message(topic, PointCloud2, timeout=20.0)

    def test_same_contract_both_sources(self):
        lightweight = self.wait_cloud("/uav1/simple_lidar/points")
        gazebo = self.wait_cloud("/ugv2/simple_lidar/points")
        for cloud in (lightweight, gazebo):
            self.assertEqual(cloud.header.frame_id, "world")
            self.assertEqual([(f.name, f.offset, f.datatype) for f in cloud.fields],
                             [("x", 0, 7), ("y", 4, 7), ("z", 8, 7)])
            self.assertEqual(cloud.point_step, 12)
            self.assertGreater(cloud.width, 50)
        a, b = sorted(xyz(lightweight)), sorted(xyz(gazebo))
        self.assertEqual(len(a), len(b))
        self.assertLess(max(math.dist(p, q) for p, q in zip(a, b)), 1e-4)
        for topic in ("/uav1/simple_lidar/beams", "/ugv2/simple_lidar/beams"):
            beams = self.wait_cloud(topic)
            self.assertEqual([f.name for f in beams.fields], ["x", "y", "z", "dx", "dy", "dz", "range", "hit"])
            self.assertEqual(beams.width, 90 * 8)

    def test_process_launch_applies_the_preset(self):
        cloud = self.wait_cloud("/uav4/simple_lidar/points")
        self.assertEqual(cloud.header.frame_id, "world")
        self.assertEqual(cloud.point_step, 12)
        points = xyz(cloud)
        self.assertGreater(len(points), 10)
        sensor, yaw = (0.5, 0.2, 1.0), 0.4
        heading = (math.cos(yaw), math.sin(yaw), 0.0)
        for p in points:
            offset = [p[i] - sensor[i] for i in range(3)]
            distance = math.sqrt(sum(v * v for v in offset))
            self.assertLessEqual(distance, 5.0 + 1e-3)  # zju_cpu_crop: 5 m see-through ball
            self.assertGreaterEqual(sum(offset[i] * heading[i] for i in range(3)) / distance, 0.5 - 1e-3)
            self.assertLessEqual(abs(offset[2]), math.tan(math.pi / 6) * 5.0 + 1e-3)
        master = rosgraph.Master("/topic_contract")
        self.assertNotIn("/uav4/simple_lidar/beams", {t for t, _ in master.getSystemState()[0]})

    def test_disabled_robot_has_no_sensor(self):
        self.wait_cloud("/ugv2/simple_lidar/points")
        master = rosgraph.Master("/topic_contract")
        publishers = {topic for topic, _ in master.getSystemState()[0]}
        nodes = {node for _, nodes in master.getSystemState()[0] for node in nodes}
        self.assertIn("/uav1/simple_lidar/points", publishers)
        self.assertIn("/ugv2/simple_lidar/points", publishers)
        self.assertFalse(any(topic.startswith("/ugv3/simple_lidar") for topic in publishers))
        self.assertNotIn("/ugv3/world_lidar", nodes)


if __name__ == "__main__":
    rospy.init_node("topic_contract")
    rostest.rosrun("xgc2_world_lidar", "topic_contract", TopicContract)
