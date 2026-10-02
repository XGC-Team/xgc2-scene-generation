#!/usr/bin/env python3
"""Exercise cached publication and fleet exclusion on real ROS nodes."""
import copy
import struct
import subprocess
import tempfile
import time
import unittest

import roslib.packages
import rospy
import rostest
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool
from xgc2_geometry_msgs.msg import SceneSnapshot

from topic_contract_test import snapshot


class CacheLifecycle(unittest.TestCase):
    def setUp(self):
        self.processes = []
        self.subscribers = []
        self.publishers = []
        self.logs = []

    def tearDown(self):
        for sub in self.subscribers:
            sub.unregister()
        for pub in self.publishers:
            pub.unregister()
        for process in self.processes:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for log in self.logs:
            log.close()

    def wait(self, predicate, detail, timeout=8):
        deadline = time.monotonic() + timeout
        while not predicate():
            self.assertTrue(all(p.poll() is None for p in self.processes), 'node exited')
            if time.monotonic() >= deadline:
                outputs = []
                for log in self.logs:
                    log.seek(0)
                    outputs.append(log.read().decode(errors='replace')[-3000:])
                self.fail(detail + '\n' + '\n'.join(outputs))
            time.sleep(0.02)

    def start(self, name, **parameters):
        executable = roslib.packages.find_node('xgc2_world_lidar', 'world_lidar_node')[0]
        parameters.setdefault('scene_namespace', '/pr6_cache')
        args = [executable, '__name:=' + name]
        # roscpp private CLI arguments are scalars; arrays must be real
        # XmlRpc parameters, otherwise vehicle_ids silently falls back to 1.
        for key, value in parameters.items():
            rospy.set_param('/' + name + '/' + key, value)
        log = tempfile.TemporaryFile()
        self.logs.append(log)
        self.processes.append(subprocess.Popen(args, stdout=log, stderr=log))
        service = '/' + name + '/set_enabled'
        rospy.wait_for_service(service, timeout=8)
        return rospy.ServiceProxy(service, SetBool)

    def listen(self, topic):
        seen = []
        self.subscribers.append(rospy.Subscriber(topic, PointCloud2, seen.append, queue_size=100))
        return seen

    def publisher(self, topic, kind):
        pub = rospy.Publisher(topic, kind, queue_size=1, latch=True)
        self.publishers.append(pub)
        return pub

    def test_disabled_reference_invalidation_waits_for_enable(self):
        toggle = self.start('pr6_map', publish_global_map=True,
                            global_map_topic='/pr6_map/cloud', num_vehicles=0)
        seen = self.listen('/pr6_map/cloud')
        source = self.publisher('/pr6_cache/snapshot', SceneSnapshot)
        self.wait(lambda: source.get_num_connections() == 1, 'snapshot did not connect')
        original = snapshot()
        source.publish(original)
        self.wait(lambda: seen and seen[-1].width > 0, 'initial reference map absent')
        self.assertTrue(toggle(False).success)
        time.sleep(0.15)
        before = len(seen)
        removed = copy.deepcopy(original)
        removed.revision = 2
        removed.obstacles = []
        source.publish(removed)
        time.sleep(0.35)
        self.assertEqual(len(seen), before, 'disabled callback published a reference cloud')
        self.assertTrue(toggle(True).success)
        self.wait(lambda: len(seen) > before and seen[-1].width == 0,
                  'reenable retained removed obstacle map')
        # Pending dynamic revisions also clear only after enabling, then wait
        # for their matching state rather than republishing an old map.
        restored = copy.deepcopy(original)
        restored.revision = 3
        source.publish(restored)
        self.wait(lambda: seen[-1].width > 0, 'restored map absent')
        toggle(False)
        time.sleep(0.15)
        before = len(seen)
        pending = copy.deepcopy(original)
        pending.revision = 4
        pending.obstacles[0].dynamic = True
        source.publish(pending)
        time.sleep(0.35)
        self.assertEqual(len(seen), before)
        toggle(True)
        self.wait(lambda: len(seen) > before and seen[-1].width == 0,
                  'pending revision retained old reference map')

    def test_unsubscribed_noise_pool_and_neighbor_exclusion(self):
        common = dict(vehicle_ids=[1, 2, 3], enabled_vehicles=[1, 2],
                      pose_topic_pattern='/pr6_pose/{id}', mode='penetrating',
                      noise_std=0.01, seed=42, surface_spacing=0.2,
                      v_fov_deg=180.0, h_fov_deg=360.0, range=5.0,
                      pose_timeout=30.0, rate=10.0)
        serial_toggle = self.start('pr6_serial', worker_threads=1, vehicle_bodies=True,
                                  output_topic_pattern='/pr6_serial/{id}/points', **common)
        self.start('pr6_pool', worker_threads=2, vehicle_bodies=True,
                   output_topic_pattern='/pr6_pool/{id}/points', **common)
        self.start('pr6_no_bodies', worker_threads=2, vehicle_bodies=False,
                   output_topic_pattern='/pr6_no_bodies/{id}/points', **common)
        source = self.publisher('/pr6_cache/snapshot', SceneSnapshot)
        poses = [self.publisher('/pr6_pose/%d' % i, PoseStamped) for i in (1, 2, 3)]
        self.wait(lambda: source.get_num_connections() == 3 and
                  all(p.get_num_connections() == 3 for p in poses), 'fleet inputs did not connect')
        empty = snapshot()
        empty.obstacles = []
        source.publish(empty)
        stamp = rospy.Time.now()
        for pub, position in zip(poses, [(0, 0, 1), (2, 0, 1), (0, 2, 1)]):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.header.stamp = stamp
            pose.pose = Pose(Point(*position), Quaternion(0, 0, 0, 1))
            pub.publish(pose)
        # The pool node remains unsubscribed while serial consumes four scans.
        # Its first seeded cloud must still equal serial's first cloud.
        serial = [self.listen('/pr6_serial/%d/points' % i) for i in (1, 2)]
        self.wait(lambda: all(len(values) >= 4 for values in serial), 'serial scans absent')
        pooled = [self.listen('/pr6_pool/%d/points' % i) for i in (1, 2)]
        absent = self.listen('/pr6_no_bodies/1/points')
        self.wait(lambda: all(len(values) >= 4 for values in pooled) and absent, 'pooled scans absent')
        for index, (a, b) in enumerate(zip(serial, pooled)):
            for left, right in zip(a[:4], b[:4]):
                self.assertEqual(left.data, right.data, 'unsubscribed node consumed scan noise')
                self.assertEqual(left.header.stamp, stamp)
                self.assertEqual(right.header.stamp, stamp)
                self.assertEqual(right.header.frame_id, 'world')
            cloud = b[0]
            self.assertEqual(cloud.point_step, 16)
            ids = {struct.unpack_from('<i', cloud.data, i * 16 + 12)[0]
                   for i in range(cloud.width)}
            self.assertEqual(ids, {1, 2, 3} - {index + 1}, 'self or unsensed neighbor semantics changed')
        self.assertEqual(absent[-1].width, 0, 'body returns appeared with vehicle_bodies=false')
        self.assertTrue(serial_toggle(False).success)
        time.sleep(0.15)
        before = [len(values) for values in serial]
        time.sleep(0.3)
        self.assertEqual([len(values) for values in serial], before, 'disabled sensor published')


if __name__ == '__main__':
    rospy.init_node('cache_lifecycle_test')
    rostest.rosrun('xgc2_world_lidar', 'cache_lifecycle', CacheLifecycle)
