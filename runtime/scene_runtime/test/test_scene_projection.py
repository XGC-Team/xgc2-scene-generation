"""The node's cached projections equal the plain ones they replace.

The envelope encoder, the per-obstacle snapshot messages and markers, and the
per-document hold states are caches. Each test compares them with the
uncached computation (written out here as the node computed it before the
caches) across edits, motion changes, play and pause.
"""

import copy
import json
import math
from pathlib import Path
import random
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_stubs  # noqa: E402

recorder = ros_stubs.install()
from xgc2_scene_runtime import ros_node  # noqa: E402
from xgc2_scene_runtime.motion import rotate  # noqa: E402
from xgc2_scene_runtime.store import EnvelopeJson  # noqa: E402


def tree(value):
    """Plain data of a stand-in message, for comparing contents."""
    if isinstance(value, ros_stubs.Time):
        return ('time', value.secs)
    if isinstance(value, ros_stubs.Msg):
        return {name: tree(item) for name, item in sorted(vars(value).items())}
    if isinstance(value, (list, tuple)):
        return [tree(item) for item in value]
    return value


# A rotation about a general axis, so capsule end caps move along all three axes.
GENERAL = [x/math.sqrt(0.3**2+0.5**2+0.2**2+0.79**2) for x in (0.3, 0.5, 0.2, 0.79)]


def ring(index, segments=12):
    parts = []
    for k in range(segments):
        a0, a1 = 2*math.pi*k/segments, 2*math.pi*(k+1)/segments
        p0, p1 = (math.cos(a0), math.sin(a0), 0.0), (math.cos(a1), math.sin(a1), 0.0)
        half = math.atan2(p1[1]-p0[1], p1[0]-p0[0])/2
        parts.append({'id': 'segment-{}'.format(k),
                      'pose': {'position': [(p0[0]+p1[0])/2, (p0[1]+p1[1])/2, 0.03*k],
                               'orientation': GENERAL if k % 3 == 2 else
                               [0.0, math.sin(0.7), 0.0, math.cos(0.7)] if k % 2 else
                               [0.0, 0.0, math.sin(half), math.cos(half)]},
                      'geometry': {'type': 'capsule', 'radius': 0.05,
                                   'height': 0.0 if k == 0 else math.dist(p0, p1)}})
    return {'id': 'ring-{}'.format(index), 'name': 'Ring é{}'.format(index),
            'pose': {'position': [index*2.0, 1.0, 1.5]}, 'parts': parts}


def scene(rings=4, pillars=6, moving=2):
    obstacles = [ring(i) for i in range(rings)]
    for i in range(pillars):
        item = {'id': 'pillar-{}'.format(i), 'name': 'Pillar "{}"'.format(i),
                'pose': {'position': [i*1.5, -2.0, 1.0]},
                'parts': [{'id': 'body', 'geometry': {'type': 'cylinder', 'radius': 0.2, 'height': 4.0}},
                          {'id': 'cap', 'pose': {'position': [0, 0, 2.2]},
                           'geometry': {'type': 'sphere', 'radius': 0.25}},
                          {'id': 'base', 'geometry': {'type': 'box', 'size': [0.5, 0.5, 0.2]}}]}
        if i < moving:
            item['motion'] = {'type': 'constant_twist', 'linear': [0.1, 0.0, 0.0], 'angular': [0.0, 0.0, 0.3]}
        obstacles.append(item)
    return {'schema': 'xgc2.scene.v1', 'id': 'projection', 'frame': 'world', 'obstacles': obstacles}


def build_node(directory, document):
    recorder.__init__()
    source = Path(directory)/'scene.yaml'
    source.write_text(yaml.safe_dump(document, allow_unicode=True))
    recorder.params.update({'~scene_file': str(source), '~gazebo': False, '~frozen': False,
                            '~save_directory': str(directory)})
    ros_node.SceneService = ros_stubs.SceneService
    return ros_node.SceneNode()


def command(node, **request):
    request.setdefault('requestId', 'r{}'.format(random.random()))
    request.update(expectedEpoch=node.store.epoch, expectedRevision=node.store.revision)
    response = node.command_value(request)
    assert response['success'], response
    return response


def reference_part_markers(node, oid, part):
    """part_markers as written before the cap markers were built directly."""
    geometry = part.geometry
    marker = ros_stubs.Marker()
    marker.header.frame_id = node.obstacle_frame(oid)
    marker.header.stamp = ros_stubs.Time(0)
    marker.ns = oid+'/'+part.id
    marker.id = 0
    marker.action = ros_stubs.Marker.ADD
    marker.pose = part.pose
    marker.color = part.color
    marker.frame_locked = True
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0
    if geometry.type == 'box':
        marker.type = ros_stubs.Marker.CUBE
        marker.scale = geometry.size
    elif geometry.type == 'sphere':
        marker.type = ros_stubs.Marker.SPHERE
        marker.scale.x = marker.scale.y = marker.scale.z = 2*geometry.radius
    elif geometry.type in ('cylinder', 'capsule'):
        marker.type = ros_stubs.Marker.CYLINDER
        marker.scale.x = marker.scale.y = 2*geometry.radius
        marker.scale.z = geometry.height
        if geometry.type == 'capsule':
            result = [marker] if geometry.height > 0 else []
            for index, direction in enumerate((-1, 1) if geometry.height else (1,)):
                cap = copy.deepcopy(marker)
                cap.id = index+1
                cap.type = ros_stubs.Marker.SPHERE
                cap.scale.x = cap.scale.y = cap.scale.z = 2*geometry.radius
                q = marker.pose.orientation
                delta = rotate([q.x, q.y, q.z, q.w], [0, 0, direction*geometry.height/2])
                cap.pose.position.x += delta[0]
                cap.pose.position.y += delta[1]
                cap.pose.position.z += delta[2]
                result.append(cap)
            return result
    elif geometry.type == 'convex':
        marker.type = ros_stubs.Marker.TRIANGLE_LIST
        marker.points = [geometry.vertices[i] for i in geometry.triangles]
    return [marker]


def reference_state(node):
    """The SceneState the tick published before hold states were cached."""
    message = ros_stubs.SceneState()
    message.epoch, message.revision = node.store.epoch, node.store.revision
    message.playing, message.scene_time = node.store.playing, node.store.scene_time()
    for item in node.store.states():
        body = ros_stubs.Msg()
        body.id = item['id']
        ros_node.set_pose(body.pose, item['pose'])
        body.twist.linear.x, body.twist.linear.y, body.twist.linear.z = item['linear']
        body.twist.angular.x, body.twist.angular.y, body.twist.angular.z = item['angular']
        message.obstacles.append(body)
    return message


class EnvelopeJsonTest(unittest.TestCase):
    def test_equals_json_dumps(self):
        encoder = EnvelopeJson()
        document = scene()
        values = [
            {'epoch': 'e', 'revision': 3, 'dirty': False, 'sceneTime': 0.1, 'document': document,
             'consumers': [{'consumer': 'ugvé\n"\\', 'applied': True}], 'online': True},
            {'document': document},
            {'document': dict(document, obstacles=[])},
            {'document': {'schema': 'x'}},
            {'document': None, 'success': False, 'error': 'bad'},
            {'success': True},
            [1, 2.5, 'x'],
            'text',
            {'document': document, 1: 'non-string key'},
            {},
        ]
        for value in values:
            with self.subTest(value=str(value)[:60]):
                self.assertEqual(encoder.dumps(value), json.dumps(value, ensure_ascii=False, allow_nan=False))
        # A new revision shares unchanged obstacles: still the same text.
        edited = dict(document, obstacles=list(document['obstacles']))
        edited['obstacles'][3] = dict(edited['obstacles'][3], name='renamed')
        del edited['obstacles'][5]
        for value in ({'document': edited, 'revision': 4}, {'document': document}, {'document': edited}):
            self.assertEqual(encoder.dumps(value), json.dumps(value, ensure_ascii=False, allow_nan=False))
        with self.assertRaises(ValueError):
            encoder.dumps({'document': document, 'sceneTime': float('nan')})


class ProjectionTest(unittest.TestCase):
    def setUp(self):
        self.now = ros_stubs.Time.current
        self.addCleanup(setattr, ros_stubs.Time, 'current', self.now)

    def test_snapshots_markers_and_states_equal_the_uncached_projections(self):
        with tempfile.TemporaryDirectory() as directory:
            node = build_node(directory, scene())

            def check(label):
                fresh = ros_node.snapshot(node.store.document, node.store.epoch, node.store.revision)
                cached = ros_node.snapshot(node.store.document, node.store.epoch, node.store.revision,
                                           node._obstacle_messages)
                self.assertEqual(tree(cached.obstacles), tree(fresh.obstacles), label)
                node.publish_definition()
                markers = recorder.last['markers'].markers
                expected = [marker for body in fresh.obstacles for part in body.parts
                            for marker in reference_part_markers(node, body.id, part)]
                self.assertEqual(tree(markers[1:]), tree(expected), label)
                self.assertEqual(markers[0].action, ros_stubs.Marker.DELETEALL)
                for _ in range(2):
                    node.tick(None)
                    self.assertEqual(tree(recorder.last['state'].obstacles),
                                     tree(reference_state(node).obstacles), label)
                self.assertEqual(recorder.last['document'].data,
                                 json.dumps(dict(node.store.status(), sceneTime=node.store.scene_time(),
                                                 document=node.store.document,
                                                 **{key: value for key, value in
                                                    json.loads(recorder.last['document'].data).items()
                                                    if key in ('consumers', 'online', 'synchronized',
                                                               'syncRetryable')}),
                                            ensure_ascii=False, allow_nan=False), label)

            check('initial')
            command(node, operation='play')
            ros_stubs.Time.current += 2.0
            check('playing')
            moved = dict(node.store.document['obstacles'][1], pose={'position': [9.0, 9.0, 1.0],
                                                                    'orientation': [0, 0, 0, 1.0]})
            command(node, operation='update', obstacle=moved)
            check('moved a ring')
            spun = dict(node.store.document['obstacles'][7])
            spun['motion'] = {'type': 'circle', 'center': [0.0, 0.0, 1.0], 'radius': 2.0,
                              'angular_speed': 0.5, 'phase': 0.0}
            spun['pose'] = {'position': [2.0, 0.0, 1.0], 'orientation': [0, 0, 0, 1.0]}
            command(node, operation='update', obstacle=spun)
            check('hold obstacle starts moving')
            command(node, operation='pause')
            ros_stubs.Time.current += 1.0
            check('paused')
            command(node, operation='delete', id='ring-2')
            check('deleted a ring')
            command(node, operation='undo')
            check('undo')


if __name__ == '__main__':
    unittest.main()
