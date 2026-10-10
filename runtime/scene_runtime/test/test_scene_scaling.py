"""Scene runtime cost for scenes with hundreds of obstacles.

SceneNode runs on ROS stand-ins with a fake Gazebo apply service that counts
calls. measure() is also the benchmark behind the numbers in the commit that
introduced this file: run this module directly to print them.
"""

import json
import math
from pathlib import Path
import sys
import tempfile
import time
import unittest

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import ros_stubs  # noqa: E402

recorder = ros_stubs.install()
from xgc2_scene_runtime import ros_node  # noqa: E402

SIZES = (50, 200, 500)
# measure(FOREST): the native forest shape (80 obstacles, 2580 parts).
FOREST = 'forest'


class CountingList(list):
    """Counts full iterations, the unit of a linear scan over obstacles."""

    scans = 0

    def __iter__(self):
        self.scans += 1
        return super().__iter__()


class FakeGazeboApply:
    """Native apply seam: one operation receives the whole document."""

    def __init__(self):
        self.calls = 0
        self.obstacles = 0

    def apply(self, document, epoch, revision):
        self.calls += 1
        self.obstacles += len(document['obstacles'])
        return {'epoch': epoch, 'revision': revision}

    def motion(self, operation, epoch, revision):
        return {'epoch': epoch, 'revision': revision}

    def close(self):
        pass


def obstacle(index, moving):
    item = {'id': 'o{:03d}'.format(index), 'pose': {'position': [(index % 25) * 2.0, (index // 25) * 2.0, 0.5]},
            'parts': [{'id': 'base', 'geometry': {'type': 'box', 'size': [0.5, 0.5, 1.0]}},
                      {'id': 'post', 'pose': {'position': [0, 0, 1.0]},
                       'geometry': {'type': 'cylinder', 'radius': 0.1, 'height': 1.0}}]}
    if moving:
        item['motion'] = {'type': 'constant_twist', 'linear': [0.1, 0.0, 0.0], 'angular': [0.0, 0.0, 0.2]}
    return item


def forest_obstacles():
    """Native Swarm-Formation forest shape: 60 pillars and 20 rings of 126
    capsule segments, 80 obstacles and 2580 parts, all held still."""
    items = [{'id': 'pillar-{}'.format(i), 'pose': {'position': [(i % 12) * 2.5 - 15.0, (i // 12) * 3.0 - 7.0, 1.0]},
              'parts': [{'id': 'body', 'geometry': {'type': 'cylinder', 'radius': 0.25, 'height': 4.0}}]}
             for i in range(60)]
    for r in range(20):
        parts = []
        for k in range(126):
            a0, a1 = 2*math.pi*k/126, 2*math.pi*(k+1)/126
            half = (a0+a1)/2
            parts.append({'id': 'segment-{}'.format(k),
                          'pose': {'position': [math.cos(half), math.sin(half), 0.0],
                                   'orientation': [math.cos(half)*math.sin(math.pi/4), math.sin(half)*math.sin(math.pi/4),
                                                   0.0, math.cos(math.pi/4)]},
                          'geometry': {'type': 'capsule', 'radius': 0.05, 'height': 2*math.sin((a1-a0)/2)}})
        items.append({'id': 'ring-{}'.format(r), 'pose': {'position': [(r % 5) * 6.0 - 12.0, (r // 5) * 3.5 - 6.0, 1.5]},
                      'parts': parts})
    return items


def build_node(directory, count):
    recorder.__init__()
    gazebo = FakeGazeboApply()
    ros_node.SimulationClient = lambda *args, **kwargs: gazebo
    ros_node.SceneService = ros_stubs.SceneService
    source = Path(directory)/'scene.yaml'
    obstacles = forest_obstacles() if count == FOREST else [obstacle(i, i % 2 == 1) for i in range(count)]
    source.write_text(yaml.safe_dump({'schema': 'xgc2.scene.v1', 'id': 'bench', 'frame': 'world',
                                      'obstacles': obstacles}))
    recorder.params.update({'~scene_file': str(source), '~gazebo': True, '~frozen': False,
                            '~save_directory': str(directory), '~simulation_service_ref_json': '{}', '~target_id': 'fixture'})
    started = time.perf_counter()
    node = ros_node.SceneNode()
    return node, gazebo, time.perf_counter()-started


def average_seconds(function, repeat):
    started = time.perf_counter()
    for _ in range(repeat):
        function()
    return (time.perf_counter()-started)/repeat


def heartbeat(node, consumer, stamp):
    return ros_stubs.Msg(consumer=consumer, epoch=node.store.epoch, revision=node.store.revision, applied=True,
                         operational=True, capability='ok', generation=1, message='',
                         header=ros_stubs.Msg(stamp=ros_stubs.Time(stamp)))


def measure(count, consumers=10, rounds=10):
    with tempfile.TemporaryDirectory() as directory:
        node, gazebo, startup = build_node(directory, count)
        startup_applies = gazebo.calls
        tick = average_seconds(lambda: node.tick(None), 20)
        definition = average_seconds(node.publish_definition, 5)
        document = average_seconds(node.publish_document, 5)
        get_seconds = average_seconds(lambda: node.command_value({'operation': 'get'}), 5)

        published = recorder.publishes.get('document', 0)
        published_bytes = recorder.published_bytes.get('document', 0)
        started = time.perf_counter()
        for round_index in range(rounds):
            for consumer in range(consumers):
                node.publish_document(changed_only=True)
        heartbeat_seconds = time.perf_counter()-started
        heartbeat_publishes = recorder.publishes.get('document', 0)-published
        heartbeat_bytes = recorder.published_bytes.get('document', 0)-published_bytes

        applies = gazebo.calls
        updates = []
        for step in range(2):
            edited = dict(node.store.document['obstacles'][step])
            edited['pose'] = {'position': [0.25+step, 0.0, 0.5], 'orientation': [0.0, 0.0, 0.0, 1.0]}
            request = {'requestId': 'move-{}'.format(step), 'expectedEpoch': node.store.epoch,
                       'expectedRevision': node.store.revision, 'operation': 'update', 'obstacle': edited}
            started = time.perf_counter()
            response = node.command_value(request)
            updates.append(time.perf_counter()-started)
            if not response['success']:
                raise AssertionError(response)
        return {
            'obstacles': count,
            'startup_s': startup,
            'startup_applies': startup_applies,
            'tick_ms': tick*1e3,
            'definition_ms': definition*1e3,
            'document_ms': document*1e3,
            'get_ms': get_seconds*1e3,
            'heartbeats': consumers*rounds,
            'heartbeat_ms': heartbeat_seconds*1e3,
            'heartbeat_document_publishes': heartbeat_publishes,
            'heartbeat_document_mb': heartbeat_bytes/1e6,
            # The first edit writes the whole working YAML; later edits re-emit
            # only the obstacle they changed.
            'update_ms': updates[0]*1e3,
            'next_update_ms': updates[1]*1e3,
            'update_applies': (gazebo.calls-applies)/len(updates),
            'update_apply_obstacles': gazebo.obstacles,
        }


class SceneScalingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.results = {count: measure(count) for count in SIZES}

    def test_gazebo_receives_one_batched_apply_per_revision(self):
        for count, result in self.results.items():
            with self.subTest(obstacles=count):
                self.assertEqual(result['startup_applies'], 1)
                self.assertEqual(result['update_applies'], 1)

    def test_unchanged_consumer_heartbeats_do_not_republish_the_document(self):
        for count, result in self.results.items():
            with self.subTest(obstacles=count):
                # Each consumer's first report changes the published view once.
                self.assertLessEqual(result['heartbeat_document_publishes'], 10)

    def test_native_failure_and_recovery_change_the_published_view(self):
        with tempfile.TemporaryDirectory() as directory:
            node, _, _ = build_node(directory, 5)
            base = recorder.publishes.get('document', 0)
            node.gazebo_failure(node.store.epoch, node.store.revision, 'outcome unknown')
            node.publish_document(changed_only=True)
            node.publish_document(changed_only=True)
            self.assertEqual(recorder.publishes['document']-base, 1)
            self.assertFalse(json.loads(recorder.last['document'].data)['synchronized'])
            node.apply_scene(node.store.document, node.store.epoch, node.store.revision)
            node.publish_document(changed_only=True)
            self.assertEqual(recorder.publishes['document']-base, 2)
            self.assertTrue(json.loads(recorder.last['document'].data)['synchronized'])
            self.assertEqual(len(json.loads(recorder.last['document'].data)['document']['obstacles']), 5)

    def test_tick_and_definition_scan_the_obstacle_list_a_constant_number_of_times(self):
        # A frame lookup used to scan every obstacle for each moving obstacle
        # (tick) and each part marker (definition): quadratic in scene size.
        for count in SIZES:
            with self.subTest(obstacles=count), tempfile.TemporaryDirectory() as directory:
                node, _, _ = build_node(directory, count)
                obstacles = CountingList(node.store.document['obstacles'])
                node.store.document = dict(node.store.document, obstacles=obstacles)
                node.tick(None)
                node.tick(None)
                self.assertLessEqual(obstacles.scans, 2*2)
                obstacles.scans = 0
                node.publish_definition()
                self.assertLessEqual(obstacles.scans, 4)


def main():
    columns = ('obstacles', 'startup_s', 'tick_ms', 'definition_ms', 'document_ms', 'get_ms', 'heartbeat_ms',
               'heartbeat_document_publishes', 'heartbeat_document_mb', 'update_ms', 'next_update_ms', 'update_applies')
    print(' '.join('{:>14}'.format(column) for column in columns))
    for count in SIZES + (FOREST,):
        result = measure(count)
        print(' '.join('{:>14.4g}'.format(result[column]) if isinstance(result[column], float)
                       else '{:>14}'.format(result[column]) for column in columns))


if __name__ == '__main__':
    if '--benchmark' in sys.argv:
        main()
    else:
        unittest.main()
