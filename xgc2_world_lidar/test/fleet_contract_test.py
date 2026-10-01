#!/usr/bin/env python3
"""One real node serves differently configured robots from one scene."""
import json
import math
import subprocess
import tempfile
import threading
import time
import unittest

import rosgraph
import roslib.packages
import rospy
import rostest
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from sensor_msgs.msg import PointCloud2
from xgc2_geometry_msgs.msg import SceneSnapshot

from topic_contract_test import snapshot, xyz


class FleetContract(unittest.TestCase):
    def test_reference_only_world_does_not_create_sensor_topics(self):
        executables = roslib.packages.find_node('xgc2_world_lidar', 'world_lidar_node')
        self.assertTrue(executables)
        observed = []
        subscriber = rospy.Subscriber('/xgc/scene/reference_cloud', PointCloud2,
                                      observed.append, queue_size=10)
        publisher = rospy.Publisher('/reference_test/snapshot', SceneSnapshot,
                                    queue_size=1, latch=True)
        manifest = {'schemaVersion': 1, 'robots': [],
                    'referenceCloud': {'surfaceSpacing': 0.1}}
        process = subprocess.Popen([executables[0], '__name:=reference_only_lidar',
                                    '_scene_namespace:=/reference_test',
                                    '_fleet_json:=' + json.dumps(manifest)],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            deadline = time.monotonic() + 10
            while publisher.get_num_connections() != 1:
                self.assertIsNone(process.poll(), 'reference-only node exited')
                self.assertLess(time.monotonic(), deadline)
                time.sleep(0.02)
            publisher.publish(snapshot())
            while not any(cloud.width for cloud in observed):
                self.assertLess(time.monotonic(), deadline, 'no actual scene cloud')
                time.sleep(0.02)
            cloud = observed[-1]
            self.assertEqual(cloud.header.frame_id, 'world')
            self.assertEqual(cloud.point_step, 12)
            self.assertEqual([field.name for field in cloud.fields], ['x', 'y', 'z'])
            publishers = rosgraph.Master(rospy.get_name()).getSystemState()[0]
            owned = [topic for topic, nodes in publishers if '/reference_only_lidar' in nodes]
            self.assertEqual(set(owned), {'/xgc/scene/reference_cloud', '/rosout'})

            removed = snapshot()
            removed.revision = 2
            removed.obstacles = []
            publisher.publish(removed)
            deadline = time.monotonic() + 5
            while observed[-1].width:
                self.assertLess(time.monotonic(), deadline, 'removed scene retained a map')
                time.sleep(0.02)

            # An unready dynamic revision must clear the previous latched map.
            restored = snapshot()
            restored.revision = 3
            publisher.publish(restored)
            while not observed[-1].width:
                self.assertLess(time.monotonic(), deadline)
                time.sleep(0.02)
            pending = snapshot()
            pending.revision = 4
            pending.obstacles[0].dynamic = True
            publisher.publish(pending)
            while observed[-1].width:
                self.assertLess(time.monotonic(), deadline, 'pending revision retained old map')
                time.sleep(0.02)
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            subscriber.unregister()
            publisher.unregister()

    def test_fleet_manifest_exceeding_linux_argv_limit(self):
        robots = [{'namespace': '/large_fleet_with_long_namespace/uav_%d' % i,
                   'mode': 'penetrating', 'rangeMeters': 8, 'rateHz': 20,
                   'hFovDeg': 360, 'vFovDeg': 180, 'hRes': 360, 'vRes': 32,
                   'surfaceSpacing': 0.1, 'keepBuried': True, 'headingCrop': False,
                   'headingCosMin': 0, 'verticalSlabTan': 0, 'publishBeams': False}
                  for i in range(512)]
        raw = json.dumps({'schemaVersion': 1, 'robots': robots})
        self.assertGreater(len(raw), 131072)
        executables = roslib.packages.find_node('xgc2_world_lidar', 'world_lidar_node')
        self.assertTrue(executables)
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json') as manifest:
            manifest.write(raw)
            manifest.flush()
            process = subprocess.Popen([executables[0], '__name:=large_fleet_lidar',
                                        '_fleet_json_file:=' + manifest.name],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            try:
                deadline = time.monotonic() + 15
                while True:
                    self.assertIsNone(process.poll(), 'real large-fleet node exited')
                    self.assertLess(time.monotonic(), deadline, 'large-fleet ROS graph not registered')
                    publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
                    if all(row['namespace'] + '/simple_lidar/points' in publishers for row in robots):
                        break
                    time.sleep(0.05)
                for row in robots:
                    self.assertEqual(publishers[row['namespace'] + '/simple_lidar/points'], ['/large_fleet_lidar'])
            finally:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()

    def test_one_scene_subscription_rates_frames_and_removal(self):
        clouds = [[], []]
        lock = threading.Lock()

        def receive(slot, message):
            with lock:
                clouds[slot].append(message)

        namespaces = ['/uav1', '/ugv2']
        subscribers = [rospy.Subscriber(ns + '/simple_lidar/points', PointCloud2,
                                       lambda m, slot=i: receive(slot, m), queue_size=100)
                       for i, ns in enumerate(namespaces)]
        pose_publishers = [rospy.Publisher('/vrpn_client_node' + ns + '/pose', PoseStamped, queue_size=1)
                           for ns in namespaces]
        scene_publisher = rospy.Publisher('/xgc/scene/snapshot', SceneSnapshot, queue_size=1, latch=True)
        timer = None
        try:
            deadline = time.monotonic() + 10
            while (scene_publisher.get_num_connections() != 1 or
                   any(p.get_num_connections() != 1 for p in pose_publishers)):
                self.assertLess(time.monotonic(), deadline, 'fleet subscriptions did not connect')
                time.sleep(0.02)
            scene_publisher.publish(snapshot())

            def publish_poses(_):
                pose = PoseStamped()
                pose.header.frame_id = 'world'
                pose.header.stamp = rospy.Time.now()
                pose.pose = Pose(Point(0.5, 0.2, 1), Quaternion(0, 0, 0, 1))
                for publisher in pose_publishers:
                    publisher.publish(pose)

            timer = rospy.Timer(rospy.Duration(0.025), publish_poses)
            deadline = time.monotonic() + 10
            while not all(clouds):
                self.assertLess(time.monotonic(), deadline, 'fleet did not produce initial clouds')
                time.sleep(0.02)
            with lock:
                for values in clouds:
                    values.clear()
            time.sleep(4.0)
            with lock:
                observed = [list(values) for values in clouds]
            # Allow scheduling jitter, but catch the former now-last_scan
            # comparison which silently halved a configured 10 Hz sampler.
            self.assertGreaterEqual(len(observed[0]), 34)
            self.assertLessEqual(len(observed[0]), 46)
            self.assertGreaterEqual(len(observed[1]), 17)
            self.assertLessEqual(len(observed[1]), 24)
            for slot, values in enumerate(observed):
                radius = [5, 9][slot]
                for cloud in values:
                    self.assertEqual(cloud.header.frame_id, 'world')
                    self.assertGreater(cloud.header.stamp.to_sec(), 0)
                    self.assertEqual(cloud.point_step, 12)
                    self.assertEqual([field.name for field in cloud.fields], ['x', 'y', 'z'])
                    self.assertGreater(cloud.width, 0)
                    for point in xyz(cloud):
                        self.assertLessEqual(math.dist(point, (0.5, 0.2, 1)), radius + 1e-3)
            publishers = rosgraph.Master(rospy.get_name()).getSystemState()[0]
            for namespace in namespaces:
                nodes = dict(publishers)[namespace + '/simple_lidar/points']
                self.assertEqual(nodes, ['/fleet_lidar'])
            self.assertFalse(any(topic.startswith('/ugv3/simple_lidar') for topic, _ in publishers))
            self.assertEqual(scene_publisher.get_num_connections(), 1)

            removed = snapshot()
            removed.revision = 2
            removed.obstacles = []
            scene_publisher.publish(removed)
            time.sleep(0.4)
            with lock:
                for values in clouds:
                    values.clear()
            time.sleep(0.4)
            with lock:
                self.assertTrue(all(clouds))
                self.assertTrue(all(message.width == 0 for values in clouds for message in values),
                                'removed obstacle geometry still produced returns')
        finally:
            if timer:
                timer.shutdown()
            for subscriber in subscribers:
                subscriber.unregister()


if __name__ == '__main__':
    rospy.init_node('fleet_contract_test')
    rostest.rosrun('xgc2_world_lidar', 'fleet_contract', FleetContract)
