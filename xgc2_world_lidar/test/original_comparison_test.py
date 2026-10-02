#!/usr/bin/env python3
"""Compare the original and candidate ROS nodes on identical live inputs."""
import copy
import os
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from xgc2_geometry_msgs.msg import SceneObstacleState, SceneSnapshot, SceneState

from cache_lifecycle_test import NodeFixture
from topic_contract_test import snapshot


class OriginalComparison(NodeFixture):
    def test_dynamic_edits_preserve_order_frame_and_stamp(self):
        original_executable = os.environ['XGC_PR6_BASE_NODE']
        for mode in ('raycast', 'penetrating', 'depth_frustum'):
            label = mode.replace('_', '')
            source_ns = '/pr6_' + label
            scene_pub = self.publisher(source_ns + '/snapshot', SceneSnapshot)
            state_pub = self.publisher(source_ns + '/state', SceneState)
            pose_pub = self.publisher(source_ns + '/pose', PoseStamped)
            streams = []
            parameters = dict(scene_namespace=source_ns, pose_topic_pattern=source_ns + '/pose',
                              mode=mode, range=12.0, surface_spacing=0.2, rate=20.0,
                              pose_timeout=30.0, h_fov_deg=90.0, v_fov_deg=90.0,
                              h_res=30, v_res=10, width=30, height=10, noise_std=0.0)
            for variant, executable in [('original', original_executable), ('candidate', None)]:
                name = 'pr6_' + label + '_' + variant
                self.start(name, executable=executable,
                           output_topic_pattern='/' + name + '/points', **parameters)
                streams.append(self.listen('/' + name + '/points'))
            self.wait(lambda: min(scene_pub.get_num_connections(), state_pub.get_num_connections(),
                                  pose_pub.get_num_connections()) == 2, 'pair inputs absent')
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.header.stamp = rospy.Time.now()
            pose.pose = Pose(Point(0, 0, 1), Quaternion(0, 0, 0, 1))
            pose_pub.publish(pose)
            definition = snapshot()
            definition.obstacles = definition.obstacles[:1]
            definition.obstacles[0].dynamic = True
            moving = SceneState(epoch=definition.epoch, revision=definition.revision)
            moving.header.frame_id = 'world'
            moving.obstacles = [SceneObstacleState(id=definition.obstacles[0].id,
                                                   pose=definition.obstacles[0].pose)]
            previous = None

            def compare(changed=True, empty=False):
                nonlocal previous
                before = [len(values) for values in streams]
                scene_pub.publish(definition)
                moving.revision = definition.revision
                moving.header.stamp = rospy.Time.now()
                state_pub.publish(moving)

                def ready():
                    # Snapshot and state are separate ROS connections; a state
                    # delivered first is correctly ignored by either node.
                    state_pub.publish(moving)
                    if not all(len(values) > count for values, count in zip(streams, before)):
                        return False
                    left, right = streams[0][-1], streams[1][-1]
                    return (left.data == right.data and (not changed or left.data != previous)
                            and (left.width == 0 if empty else left.width > 0))

                self.wait(ready, mode + ': original/candidate geometry differs')
                left, right = streams[0][-1], streams[1][-1]
                for cloud in (left, right):
                    self.assertEqual(cloud.header.frame_id, 'world')
                    self.assertEqual(cloud.header.stamp, pose.header.stamp)
                    self.assertEqual(cloud.point_step, 12)
                    self.assertEqual(cloud.row_step, cloud.width * 12)
                self.assertEqual(left.width, right.width)
                self.assertEqual(left.fields, right.fields)
                self.assertEqual(left.data, right.data, mode + ': point order differs')
                previous = left.data

            compare()
            moving.obstacles[0].pose = copy.deepcopy(moving.obstacles[0].pose)
            moving.obstacles[0].pose.position.x += 1.0
            compare()
            definition.revision += 1
            definition.obstacles[0].parts[0].geometry.size.y = 3.0
            compare()
            added = snapshot().obstacles[2]
            added.pose.position = Point(2, 0, 1)
            added.parts[0].geometry.radius = 0.4
            definition.revision += 1
            definition.obstacles.append(added)
            compare()
            definition.revision += 1
            definition.obstacles.reverse()
            compare(changed=False)
            definition.revision += 1
            definition.obstacles = [item for item in definition.obstacles if item.dynamic]
            compare()
            definition.revision += 1
            definition.obstacles = []
            moving.obstacles = []
            compare(empty=True)


if __name__ == '__main__':
    rospy.init_node('original_comparison_test')
    rostest.rosrun('xgc2_world_lidar', 'original_comparison', OriginalComparison)
