#!/usr/bin/env python3
"""The shared static-cloud entry publishes the same bytes at every worker_threads setting."""
import math
import random
import struct
import threading
import time
import unittest

import rosgraph
import rospy
import rostest
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud2, PointField

ROBOTS = 12
VARIANTS = ('scc_t1', 'scc_t4', 'scc_default', 'scc_full', 'scc_fov')
SAME_AS_SERIAL = ('scc_t4', 'scc_default', 'scc_full')


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


def records(payloads):
    for payload in payloads:
        return [struct.unpack_from('<fff', payload, o) for o in range(0, len(payload), 16)]
    return []


def window_state(point, pose, guard):
    """+1 clearly inside, -1 clearly outside, 0 within guard of a limit (60 x 40 deg, 1 m)."""
    p = pose.position
    q = pose.orientation
    dx, dy, dz = point[0] - p.x, point[1] - p.y, point[2] - p.z
    yaw = 2 * math.atan2(q.z, q.w)  # the test poses are pure yaw
    lx = math.cos(yaw) * dx + math.sin(yaw) * dy
    ly = -math.sin(yaw) * dx + math.cos(yaw) * dy
    margins = [math.sqrt(dx * dx + dy * dy + dz * dz) - 1.0,
               math.radians(30) - abs(math.atan2(ly, lx)),
               math.radians(20) - abs(math.atan2(dz, math.hypot(lx, ly)))]
    if min(margins) > guard:
        return 1
    if min(margins) < -guard:
        return -1
    return 0


class SharedCloudCpu(unittest.TestCase):
    def test_zero_worker_threads_is_refused(self):
        # worker_threads counts the caller, so 0 is invalid rather than automatic: that node
        # exits during construction, before it advertises any output. (A node that exits
        # without unregistering can linger in the master's node list, so look at the topics.)
        master = rosgraph.Master(rospy.get_name())
        deadline = time.monotonic() + 30
        while '/scc_t1/sensor/uav1/points' not in dict(master.getSystemState()[0]):
            self.assertLess(time.monotonic(), deadline, 'the valid nodes never advertised')
            time.sleep(0.1)
        time.sleep(1.0)
        published = dict(master.getSystemState()[0])
        self.assertIn('/scc_default/sensor/uav1/points', published)
        self.assertNotIn('/scc_zero/sensor/uav1/points', published)

    def check_window(self, index, default_payloads, window_payloads):
        default, window = records(default_payloads), records(window_payloads)
        pose = pose_of(index)
        inside = [pt for pt in default if window_state(pt, pose, 1e-4) == 1]
        outside = [pt for pt in default if window_state(pt, pose, 1e-4) == -1]
        kept = [pt for pt in window if window_state(pt, pose, 1e-4) != 0]
        # In the default order, every default point clearly inside survives and every one
        # clearly outside is gone; nothing appears that the default crop did not have.
        self.assertEqual(inside, kept, 'uav%d window differs' % index)
        self.assertTrue(set(window) <= set(default), 'uav%d window adds points' % index)
        self.assertFalse(set(outside) & set(window), 'uav%d keeps points outside' % index)

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
        published = default_points = window_points = 0
        for i in range(1, ROBOTS + 1):
            with lock:
                streams = {v: list(received[(v, i)]) for v in VARIANTS}
            # The pose of each robot never changes, so every message of a topic is one payload.
            payloads = {v: {bytes(m.data) for m in streams[v]} for v in VARIANTS}
            for v in VARIANTS:
                self.assertLessEqual(len(payloads[v]), 1, '%s uav%d changed payload' % (v, i))
            for v in SAME_AS_SERIAL:
                self.assertEqual(payloads['scc_t1'], payloads[v], 'uav%d %s differs' % (i, v))
            self.check_window(i, payloads['scc_t1'], payloads['scc_fov'])
            default_points += len(records(payloads['scc_t1']))
            window_points += len(records(payloads['scc_fov']))
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
        # Publishing does not wait for a subscriber: a topic that has been running for a while
        # greets a new subscriber with a sequence number above zero (roscpp counts every
        # publish() call, connected or not).
        index = next(i for i in range(1, ROBOTS + 1) if received[('scc_t1', i)])
        late = []
        subscriber = rospy.Subscriber('/scc_t1/sensor/uav%d/points' % index, PointCloud2,
                                      late.append, queue_size=5)
        deadline = time.monotonic() + 10
        while not late:
            self.assertLess(time.monotonic(), deadline, 'no cloud for the late subscriber')
            time.sleep(0.05)
        subscriber.unregister()
        self.assertGreaterEqual(late[0].header.seq, 3)
        self.assertGreater(window_points, 0, 'the window keeps no points anywhere')
        self.assertLess(window_points, default_points, 'the window removes nothing')


if __name__ == '__main__':
    rospy.init_node('shared_cloud_cpu_test')
    rostest.rosrun('xgc2_world_lidar', 'shared_cloud_cpu', SharedCloudCpu)
