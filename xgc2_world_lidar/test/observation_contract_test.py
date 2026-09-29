#!/usr/bin/env python3
"""Exercise the actual ROS node, not a replacement sensor or map."""
import copy
import struct
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2
from xgc2_geometry_msgs.msg import (SceneGeometry, SceneObstacle, SceneObstacleState,
                                    ScenePart, SceneSnapshot, SceneState)


class ObservationContract(unittest.TestCase):
    def test_frames_stamps_mount_and_scene_epoch(self):
        seen, beams = [], []
        lock = threading.Lock()
        def receive(target, msg):
            with lock:
                target.append(msg)
        subs = [rospy.Subscriber('/lidar_contract_points', PointCloud2, lambda m: receive(seen, m)),
                rospy.Subscriber('/lidar_contract_beams', PointCloud2, lambda m: receive(beams, m))]
        pose_pub = rospy.Publisher('/lidar_contract_pose', PoseStamped, queue_size=1)
        snap_pub = rospy.Publisher('/lidar_contract_scene/snapshot', SceneSnapshot, queue_size=1, latch=True)
        state_pub = rospy.Publisher('/lidar_contract_scene/state', SceneState, queue_size=1)
        deadline = time.monotonic() + 10
        while min(pose_pub.get_num_connections(), snap_pub.get_num_connections(), state_pub.get_num_connections()) == 0:
            self.assertLess(time.monotonic(), deadline, 'real node subscriptions did not connect')
            time.sleep(0.02)
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.pose = Pose(Point(0,0,1), Quaternion(0,0,0,1))
        part = ScenePart(id='p', pose=Pose(Point(), Quaternion(0,0,0,1)),
                         geometry=SceneGeometry(type='box', size=Vector3(1,4,2)))
        obstacle = SceneObstacle(id='wall', name='wall', pose=Pose(Point(4,0,1), Quaternion(0,0,0,1)),
                                 parts=[part], dynamic=False, motion_type='static')
        snap = SceneSnapshot(scene_id='test', epoch='first', revision=1, obstacles=[obstacle])
        snap.header.frame_id = 'world'
        snap_pub.publish(snap)
        def send(p=None):
            message = copy.deepcopy(p if p is not None else pose)
            if p is None:
                message.header.stamp = rospy.Time.now()
            pose_pub.publish(message)
            return message
        def await_new():
            with lock:
                before = len(seen)
            limit = time.monotonic() + 4
            while time.monotonic() < limit:
                send()
                time.sleep(0.04)
                with lock:
                    if len(seen) > before:
                        return seen[-1]
            self.fail('no new cloud from actual node')
        first = await_new()
        self.assertEqual(first.header.frame_id, 'world')
        self.assertGreater(first.header.stamp.to_sec(), 0)
        time.sleep(0.1)
        with lock:
            self.assertTrue(beams)
            origin = struct.unpack_from('<fff', beams[-1].data)
        self.assertEqual(origin, (1.0, 0.0, 1.0))
        def assert_silent(p=None, keep_valid_pose=False):
            time.sleep(0.15)
            with lock:
                before = len(seen)
            until = time.monotonic() + 0.3
            while time.monotonic() < until:
                if keep_valid_pose:
                    send()
                elif p is not None:
                    pose_pub.publish(p)
                time.sleep(0.025)
            with lock:
                self.assertEqual(len(seen), before, 'rejected input still generated scans')
        invalid = copy.deepcopy(pose)
        invalid.header.stamp = rospy.Time.now()
        invalid.header.frame_id = 'map'
        pose_pub.publish(invalid)
        assert_silent(invalid)
        invalid.header.frame_id = 'world'
        invalid.pose.orientation = Quaternion(0,0,0,0)
        pose_pub.publish(invalid)
        assert_silent(invalid)
        future = copy.deepcopy(pose)
        future.header.stamp = rospy.Time.now() + rospy.Duration(10)
        pose_pub.publish(future)
        assert_silent(future)
        zero = copy.deepcopy(pose)
        zero.header.stamp = rospy.Time(0)
        pose_pub.publish(zero)
        assert_silent(zero)
        await_new()
        snap.epoch = 'second'
        snap.obstacles[0].dynamic = True
        snap.obstacles[0].motion_type = 'scripted'
        snap_pub.publish(snap)
        assert_silent(keep_valid_pose=True)
        state = SceneState(epoch='first', revision=1)
        state.header.frame_id = 'world'
        state.header.stamp = rospy.Time.now()
        state_pub.publish(state)
        assert_silent(keep_valid_pose=True)
        state.epoch = 'second'
        state.obstacles = [SceneObstacleState(id='wall', pose=obstacle.pose)]
        state.header.stamp = rospy.Time.now()
        state_pub.publish(state)
        await_new()
        snap_pub.publish(snap)
        state.header.stamp = rospy.Time.now()
        state_pub.publish(state)
        await_new()
        time.sleep(0.6)
        assert_silent(keep_valid_pose=True)
        for sub in subs:
            sub.unregister()


if __name__ == '__main__':
    rospy.init_node('observation_contract_test')
    rostest.rosrun('xgc2_world_lidar', 'observation_contract', ObservationContract)
