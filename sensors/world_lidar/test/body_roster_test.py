#!/usr/bin/env python3
"""Real world-model roster: an unsensed body occludes and outlives providers."""
import json
import struct
import time

import rosgraph
import rospy
import rostest
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from xgc2_geometry_msgs.msg import SceneSnapshot, SceneGeometry, SceneObstacle, ScenePart

from cache_lifecycle_test import NodeFixture


class BodyRoster(NodeFixture):
    def test_unsensed_body_wall_self_provider_stop_and_unknown_truth(self):
        bodies = [{'id': 41, 'namespace': '/A', 'poseTopic': '/p1_truth/A',
                   'geometry': {'type': 'sphere', 'radiusMeters': .8}},
                  {'id': 7, 'namespace': '/B', 'poseTopic': '/p1_truth/B',
                   'geometry': {'type': 'sphere', 'radiusMeters': .4}}]
        robot = {'namespace': '/A', 'bodyId': 41, 'mode': 'raycast', 'rateHz': 20,
                 'rangeMeters': 10, 'hRes': 1, 'vRes': 1, 'hFovDeg': 1, 'vFovDeg': 0}
        self.start('p1_raycast', fleet_json=json.dumps({'schemaVersion': 1, 'robots': [robot], 'bodies': bodies}),
                   scene_namespace='/p1_scene', pose_timeout=.4, worker_threads=1)
        cloud = self.listen('/A/simple_lidar/points')
        scene_pub = self.publisher('/p1_scene/snapshot', SceneSnapshot)
        truth = [self.publisher('/p1_truth/'+name, PoseStamped) for name in ('A', 'B')]
        provider = self.publisher('/vrpn_client_node/B/pose', PoseStamped)
        scene = SceneSnapshot(scene_id='p1', epoch='p1', revision=1)
        scene.header.frame_id = 'world'
        self.wait(lambda: scene_pub.get_num_connections() and all(p.get_num_connections() for p in truth), 'truth inputs absent')
        scene_pub.publish(scene)
        publish_b = [True]
        stamp = [None]
        def publish(_):
            stamp[0] = rospy.Time.now()
            for index, pub in enumerate(truth):
                if index and not publish_b[0]:
                    continue
                pose = PoseStamped()
                pose.header.frame_id = 'world'
                pose.header.stamp = stamp[0]
                pose.pose = Pose(Point(3*index, 0, 1), Quaternion(0, 0, 0, 1))
                pub.publish(pose)
        timer = rospy.Timer(rospy.Duration(.025), publish)
        self.addCleanup(timer.shutdown)
        def on_b():
            return cloud and cloud[-1].width == 1 and struct.unpack_from('<i', cloud[-1].data, 12)[0] == 7
        self.wait(on_b, 'body B without sensor was omitted or self A was not excluded')
        self.assertAlmostEqual(struct.unpack_from('<f', cloud[-1].data)[0], 2.6, places=5)
        self.assertEqual(cloud[-1].header.frame_id, 'world')
        self.assertEqual(cloud[-1].point_step, 16)
        publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        self.assertNotIn('/B/simple_lidar/points', publishers)
        # No provider connection or provider lifetime can erase a world body.
        self.assertEqual(provider.get_num_connections(), 0)
        provider.unregister()
        before = len(cloud)
        self.wait(lambda: len(cloud) > before and on_b(), 'provider stop erased extant world model B')
        # Static wall blocks the true B hit; it is not a body sample crop.
        wall = SceneObstacle(id='wall', pose=Pose(Point(1.5, 0, 1), Quaternion(0, 0, 0, 1)),
                             parts=[ScenePart(id='p', pose=Pose(Point(), Quaternion(0, 0, 0, 1)),
                                              geometry=SceneGeometry(type='box', size=Vector3(.2, 4, 3)))])
        scene.revision += 1
        scene.obstacles = [wall]
        scene_pub.publish(scene)
        self.wait(lambda: cloud and cloud[-1].width == 1 and
                  struct.unpack_from('<i', cloud[-1].data, 12)[0] == -1, 'wall did not occlude B')
        self.assertAlmostEqual(struct.unpack_from('<f', cloud[-1].data)[0], 1.4, places=5)
        scene.revision += 1
        scene.obstacles = []
        scene_pub.publish(scene)
        self.wait(on_b, 'wall removal did not restore B')
        # Missing body truth fails closed; it cannot become free space or a
        # shorter roster. Resume of the same model restores its same body id.
        publish_b[0] = False
        time.sleep(.6)
        before = len(cloud)
        time.sleep(.2)
        self.assertEqual(len(cloud), before, 'stale B silently disappeared from observation')
        publish_b[0] = True
        self.wait(lambda: len(cloud) > before and on_b(), 'truth resume did not restore B')

    def test_penetrating_roster_does_not_add_neighbor_baseline(self):
        manifest = {'schemaVersion': 1, 'robots': [{'namespace': '/crop_A', 'bodyId': 4,
                    'poseTopic': '/p1_crop/A', 'mode': 'penetrating', 'rateHz': 10,
                    'rangeMeters': 8, 'hFovDeg': 360, 'vFovDeg': 180, 'headingCrop': True, 'headingCosMin': .5}],
                    'bodies': [{'id': 4, 'namespace': '/crop_A', 'poseTopic': '/p1_crop/A',
                                'geometry': {'type': 'sphere', 'radiusMeters': .4}},
                               {'id': 9, 'namespace': '/crop_B', 'poseTopic': '/p1_crop/B',
                                'geometry': {'type': 'sphere', 'radiusMeters': .4}}]}
        self.start('p1_crop', fleet_json=json.dumps(manifest), scene_namespace='/p1_crop_scene', pose_timeout=10)
        cloud = self.listen('/crop_A/simple_lidar/points')
        scene_pub = self.publisher('/p1_crop_scene/snapshot', SceneSnapshot)
        a = self.publisher('/p1_crop/A', PoseStamped)
        b = self.publisher('/p1_crop/B', PoseStamped)
        self.wait(lambda: scene_pub.get_num_connections() and a.get_num_connections() and b.get_num_connections(), 'crop inputs absent')
        scene = SceneSnapshot(scene_id='crop', epoch='crop', revision=1)
        scene.header.frame_id = 'world'
        scene_pub.publish(scene)
        for pub, x in ((a, 0), (b, 2)):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.header.stamp = rospy.Time.now()
            pose.pose = Pose(Point(x, 0, 1), Quaternion(0, 0, 0, 1))
            pub.publish(pose)
        self.wait(lambda: cloud, 'crop cloud absent')
        self.assertTrue(all(msg.width == 0 and msg.point_step == 12 for msg in cloud),
                        'body roster silently changed penetrating/crop baseline')


if __name__ == '__main__':
    rospy.init_node('body_roster_test')
    rostest.rosrun('xgc2_world_lidar', 'body_roster', BodyRoster)
