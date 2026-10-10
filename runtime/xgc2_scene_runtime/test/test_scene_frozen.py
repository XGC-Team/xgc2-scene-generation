import copy
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml

from xgc2_scene_runtime.store import FROZEN_ERROR, FROZEN_REJECTED, SceneStore, load

LAUNCH = Path(__file__).resolve().parents[1]/'launch'/'scene.launch'


def box(oid='box'):
    return {'id': oid, 'pose': {'position': [1, 2, 3]}, 'parts': [
        {'id': 'body', 'geometry': {'type': 'box', 'size': [1, 2, 3]}}]}


def scene(*obstacles):
    return {'schema': 'xgc2.scene.v1', 'id': 'test', 'frame': 'world', 'obstacles': list(obstacles)}


def command(store, operation, **values):
    return store.command(dict(requestId='r{}'.format(len(store.requests)), expectedEpoch=store.epoch,
                              expectedRevision=store.revision, operation=operation, **values))


class FrozenSceneStoreTest(unittest.TestCase):
    def test_every_edit_save_and_reload_channel_is_rejected_with_an_explicit_error(self):
        applied = []
        store = SceneStore(scene(box()), apply=lambda doc, epoch, revision: applied.append(revision),
                           frozen=True)
        self.assertEqual(applied, [1])
        for operation, values in [('add', {'obstacle': box('new')}), ('update', {'obstacle': box()}),
                                  ('delete', {'id': 'box'}), ('clear', {}),
                                  ('replace', {'document': scene(box('other'))}),
                                  ('undo', {}), ('redo', {}), ('save', {}), ('reload', {})]:
            with self.subTest(operation=operation):
                self.assertIn(operation, FROZEN_REJECTED)
                before = copy.deepcopy(store.envelope())
                result = command(store, operation, **values)
                self.assertFalse(result['success'])
                self.assertEqual(result['error'], FROZEN_ERROR)
                self.assertEqual(store.revision, 1)
                self.assertEqual(store.document, before['document'])
                self.assertEqual(store.undo_stack, [])
        self.assertEqual(applied, [1])

    def test_rejection_is_a_result_not_a_silent_drop_or_exception(self):
        store = SceneStore(scene(box()), frozen=True)
        result = command(store, 'clear')
        self.assertEqual(result['success'], False)
        self.assertIn('read-only', result['error'])
        self.assertIn('editable scene', result['error'])
        replay = store.command(dict(requestId='r0', expectedEpoch=store.epoch, expectedRevision=1,
                                    operation='clear'))
        self.assertEqual(replay, result)

    def test_frozen_scene_still_serves_reads_motion_clock_and_resync(self):
        applied = []
        store = SceneStore(scene(box()), apply=lambda doc, epoch, revision: applied.append((revision, doc)),
                           frozen=True)
        self.assertEqual(store.command({'operation': 'get'})['document']['obstacles'][0]['id'], 'box')
        for operation in ('play', 'pause', 'reset'):
            with self.subTest(operation=operation):
                self.assertTrue(command(store, operation)['success'], operation)
        self.assertTrue(command(store, 'resync')['success'])
        self.assertEqual(store.revision, 2)
        self.assertEqual(applied[-1], (2, store.document))
        self.assertTrue(store.envelope()['frozen'])

    def test_frozen_source_yaml_is_never_reloaded_or_written(self):
        with tempfile.TemporaryDirectory() as root:
            source = Path(root)/'scene.yaml'
            source.write_text(yaml.safe_dump(scene(box())))
            store = SceneStore(load(source)[0], source=source, frozen=True)
            external = yaml.safe_dump(scene(box('agent')))
            source.write_text(external)
            self.assertFalse(command(store, 'reload')['success'])
            self.assertFalse(command(store, 'save')['success'])
            self.assertEqual(store.document['obstacles'][0]['id'], 'box')
            self.assertEqual(source.read_text(), external)
            self.assertEqual(load(source)[0]['obstacles'][0]['id'], 'agent')

    def test_unfrozen_default_keeps_every_existing_channel_open(self):
        with tempfile.TemporaryDirectory() as root:
            source = Path(root)/'scene.yaml'
            source.write_text(yaml.safe_dump(scene(box())))
            store = SceneStore(load(source)[0], source=source)
            self.assertFalse(store.envelope()['frozen'])
            self.assertTrue(command(store, 'add', obstacle=box('new'))['success'])
            self.assertTrue(command(store, 'undo')['success'])
            self.assertTrue(command(store, 'save')['success'])
            source.write_text(yaml.safe_dump(scene(box('agent'))))
            self.assertTrue(command(store, 'reload')['success'])
            self.assertEqual(store.document['obstacles'][0]['id'], 'agent')


class FrozenLaunchContractTest(unittest.TestCase):
    def test_scene_launch_forwards_the_frozen_argument_as_a_param(self):
        root = ET.parse(str(LAUNCH)).getroot()
        args = {item.get('name'): item.get('default') for item in root.iter('arg')}
        self.assertEqual(args.get('frozen'), 'false')
        params = {item.get('name'): item.get('value') for item in root.iter('param')}
        self.assertEqual(params.get('frozen'), '$(arg frozen)')
        self.assertEqual(args.get('working_file'), '')
        self.assertEqual(params.get('working_file'), '$(arg working_file)')


if __name__ == '__main__':
    unittest.main()
