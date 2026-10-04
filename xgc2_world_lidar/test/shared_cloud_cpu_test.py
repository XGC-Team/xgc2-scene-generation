#!/usr/bin/env python3
"""The shared static-cloud entry publishes the same bytes at every worker_threads setting."""
import math
import random
import struct
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud2, PointField

ROBOTS = 12
VARIANTS = ('scc_t1', 'scc_t4', 'scc_auto')


def world_cloud():
    """Dense 0.1 m columns on a lattice plus a few rings: the native Swarm world's structure."""
    rng = random.Random(3)
    points = []
    for _ in range(40):
        x = math.floor(rng.uniform(-12, 12) * 10) / 10
        y = math.floor(rng.uniform(-6, 6) * 10) / 10
        for i in range(-2, 3):
            for j in range(-2, 3):
                for k in range(35):
                    points.append((x + 0.1 * i, y + 0.1 * j, -1.0 + 0.1 * k))
    msg = PointCloud2()
    msg.header.frame_id = 'map'
    msg.height = 1
    msg.width = len(points)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b''.join(struct.pack('<fff', *p) for p in points)
    return msg


def pose_of(index):
    rng = random.Random(100 + index)
    yaw = rng.uniform(-math.pi, math.pi)
    return Pose(Point(rng.uniform(-12, 12), rng.uniform(-6, 6), rng.uniform(0.3, 2.0)),
                Quaternion(0, 0, math.sin(yaw / 2), math.cos(yaw / 2)))


class SharedCloudCpu(unittest.TestCase):
    def test_every_worker_setting_publishes_the_same_bytes(self):
        world = rospy.Publisher('/scc_world', PointCloud2, queue_size=1, latch=True)
        poses = [rospy.Publisher('/scc_pose/uav%d' % i, PoseStamped, queue_size=1)
                 for i in range(1, ROBOTS + 1)]
        received = {(v, i): [] for v in VARIANTS for i in range(1, ROBOTS + 1)}
        lock = threading.Lock()
        subscribers = []
        for v in VARIANTS:
            for i in range(1, ROBOTS + 1):
                def keep(msg, key=(v, i)):
                    with lock:
                        received[key].append(msg)
                subscribers.append(rospy.Subscriber('/%s/sensor/uav%d/points' % (v, i),
                                                    PointCloud2, keep, queue_size=50))
        deadline = time.monotonic() + 30
        while (world.get_num_connections() < len(VARIANTS)
               or any(p.get_num_connections() < len(VARIANTS) for p in poses)):
            self.assertLess(time.monotonic(), deadline, 'node inputs never connected')
            time.sleep(0.05)
        world.publish(world_cloud())
        stop = threading.Event()

        def publish_poses():
            while not stop.is_set():
                for i, publisher in enumerate(poses, 1):
                    msg = PoseStamped()
                    msg.header.frame_id = 'map'
                    msg.header.stamp = rospy.Time.now()
                    msg.pose = pose_of(i)
                    publisher.publish(msg)
                time.sleep(0.05)

        thread = threading.Thread(target=publish_poses)
        thread.start()
        try:
            def settled():
                with lock:
                    return all(len(received[(v, i)]) >= 3
                               for v in VARIANTS for i in range(1, ROBOTS + 1)
                               if received[('scc_t1', i)])
            deadline = time.monotonic() + 30
            while not (settled() and any(received[('scc_t1', i)] for i in range(1, ROBOTS + 1))):
                self.assertLess(time.monotonic(), deadline, 'no clouds published')
                time.sleep(0.1)
            time.sleep(1.0)
        finally:
            stop.set()
            thread.join()
        published = 0
        for i in range(1, ROBOTS + 1):
            with lock:
                streams = {v: list(received[(v, i)]) for v in VARIANTS}
            # The pose of each robot never changes, so every message of a topic is one payload.
            payloads = {v: {bytes(m.data) for m in streams[v]} for v in VARIANTS}
            for v in VARIANTS:
                self.assertLessEqual(len(payloads[v]), 1, '%s uav%d changed payload' % (v, i))
            self.assertEqual(payloads['scc_t1'], payloads['scc_t4'], 'uav%d t4 differs' % i)
            self.assertEqual(payloads['scc_t1'], payloads['scc_auto'], 'uav%d auto differs' % i)
            for v in VARIANTS:
                for m in streams[v]:
                    self.assertEqual(m.header.frame_id, 'map')
                    self.assertEqual(m.header.stamp, rospy.Time(0))
                    self.assertEqual(m.point_step, 16)  # PCL PointXYZ: x y z and one pad float
                    self.assertEqual(len(m.data), 16 * m.width)
            published += 1 if payloads['scc_t1'] else 0
        # Robots with no map point in range publish nothing (the original no-publication rule),
        # so most, not necessarily all, of the twelve have a cloud.
        self.assertGreaterEqual(published, 6)


if __name__ == '__main__':
    rospy.init_node('shared_cloud_cpu_test')
    rostest.rosrun('xgc2_world_lidar', 'shared_cloud_cpu', SharedCloudCpu)
