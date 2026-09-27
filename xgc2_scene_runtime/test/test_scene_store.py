import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import yaml

from xgc2_scene_runtime.document import SceneError, document, geometry
from xgc2_scene_runtime.motion import state
from xgc2_scene_runtime.store import SceneStore, load, unique_object


def box(oid='box'):
    return {'id': oid, 'pose': {'position': [1, 2, 3]}, 'parts': [
        {'id': 'body', 'geometry': {'type': 'box', 'size': [1, 2, 3]}}]}


def scene(*obstacles):
    return {'schema': 'xgc2.scene.v1', 'id': 'test', 'frame': 'world', 'obstacles': list(obstacles)}


def command(store, operation, **values):
    return store.command(dict(requestId='r{}'.format(len(store.requests)), expectedEpoch=store.epoch,
                              expectedRevision=store.revision, operation=operation, **values))


class SceneStoreTest(unittest.TestCase):
    def test_obstacle_visual_color_is_product_amber(self):
        from xgc2_scene_runtime.document import OBSTACLE_VISUAL_COLOR
        gray = box()
        gray['parts'][0]['color'] = [0.5, 0.5, 0.5, 1.0]
        self.assertEqual(document(scene(gray))['obstacles'][0]['parts'][0]['color'],
                         OBSTACLE_VISUAL_COLOR)
        self.assertEqual(document(scene(box()))['obstacles'][0]['parts'][0]['color'],
                         OBSTACLE_VISUAL_COLOR)

    def test_compounds_preserve_parts_and_geometry_has_explicit_sizes(self):
        arch = box('arch')
        arch['parts'] = [dict(id='left', geometry={'type': 'box', 'size': [0.2, 1, 2]}),
                         dict(id='right', pose={'position': [2, 0, 0]}, geometry={'type': 'sphere', 'radius': 0.2}),
                         dict(id='top', pose={'position': [1, 0, 2]}, geometry={'type': 'capsule', 'radius': 0.2, 'height': 2})]
        normalized = document(scene(arch))
        self.assertEqual(len(normalized['obstacles'][0]['parts']), 3)
        self.assertEqual(normalized['obstacles'][0]['parts'][2]['geometry']['height'], 2)
        self.assertEqual(normalized['obstacles'][0]['parts'][1]['pose']['position'], [2, 0, 0])

    def test_invalid_geometry_does_not_become_another_shape(self):
        for invalid in [{'type': 'cone', 'radius': 2}, {'type': 'sphere', 'radius': -1},
                        {'type': 'cylinder', 'radius': 1, 'height': 0},
                        {'type': 'box', 'size': [1, float('nan'), 1]},
                        {'type': 'box', 'size': [True, 1, 1]}]:
            with self.subTest(invalid=invalid), self.assertRaises(SceneError):
                geometry(invalid)

    def test_duplicate_fields_and_huge_numbers_are_rejected(self):
        with self.assertRaises(SceneError):
            json.loads('{"operation":"get","operation":"clear"}', object_pairs_hook=unique_object)
        with self.assertRaises(SceneError):
            geometry({'type': 'sphere', 'radius': 10**1000})
        with tempfile.TemporaryDirectory() as root:
            source = Path(root)/'scene.yaml'
            source.write_text('schema: xgc2.scene.v1\nid: first\nid: second\nframe: world\nobstacles: []\n')
            with self.assertRaises(SceneError):
                load(source)

    def test_convex_manifold_required_without_filling_compound_holes(self):
        tetra = {'type': 'convex', 'vertices': [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
                 'triangles': [0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2]}
        self.assertEqual(geometry(tetra)['vertices'], tetra['vertices'])
        open_mesh = copy.deepcopy(tetra)
        open_mesh['triangles'][-3:] = open_mesh['triangles'][:3]
        with self.assertRaises(SceneError):
            geometry(open_mesh)

    def test_ids_are_stable_through_delete_copy_undo_and_reload(self):
        store = SceneStore(scene(box('a'), box('b'), box('c')))
        self.assertTrue(command(store, 'delete', id='b')['success'])
        self.assertEqual([x['id'] for x in store.document['obstacles']], ['a', 'c'])
        self.assertTrue(command(store, 'undo')['success'])
        self.assertEqual([x['id'] for x in store.document['obstacles']], ['a', 'b', 'c'])
        self.assertFalse(store.envelope()['dirty'])
        self.assertTrue(command(store, 'redo')['success'])
        self.assertTrue(command(store, 'add', obstacle=box('copy'))['success'])
        self.assertFalse(command(store, 'redo')['success'])

    def test_stale_epoch_revision_and_request_reuse_cannot_overwrite(self):
        store = SceneStore(scene(box()))
        first = dict(requestId='same', expectedEpoch=store.epoch, expectedRevision=1, operation='add', obstacle=box('new'))
        result = store.command(first)
        self.assertTrue(result['success'])
        self.assertEqual(store.command(first), result)
        self.assertEqual(len(store.document['obstacles']), 2)
        self.assertFalse(store.command(dict(first, operation='clear'))['success'])
        self.assertFalse(store.command(dict(first, requestId='stale'))['success'])
        self.assertFalse(store.command(dict(first, requestId='epoch', expectedRevision=2, expectedEpoch='old'))['success'])

    def test_simulator_rejection_does_not_commit_or_create_undo_entry(self):
        applied = []
        def apply(doc, epoch, revision):
            if revision > 1:
                raise SceneError('collision update rejected')
            applied.append(revision)
        store = SceneStore(scene(box()), apply=apply)
        result = command(store, 'clear')
        self.assertFalse(result['success'])
        self.assertEqual(store.revision, 1)
        self.assertEqual(len(store.document['obstacles']), 1)
        self.assertEqual(store.undo_stack, [])

    def test_edits_undo_and_redo_automatically_survive_restart(self):
        with tempfile.TemporaryDirectory() as root:
            source = Path(root)/'scene.yaml'
            source.write_text(yaml.safe_dump(scene(box())))
            source.chmod(0o640)
            store = SceneStore(load(source)[0], source=source)
            for operation, values in [('add', {'obstacle': box('new')}), ('undo', {}), ('redo', {})]:
                result = command(store, operation, **values)
                self.assertTrue(result['success'], result)
                self.assertFalse(result['dirty'])
                self.assertEqual(result['savedRevision'], result['revision'])
                self.assertEqual(SceneStore(load(source)[0], source=source).document, store.document)
            self.assertEqual(source.stat().st_mode & 0o777, 0o640)

    def test_agent_file_edits_require_reload_and_are_never_overwritten(self):
        with tempfile.TemporaryDirectory() as root:
            source = Path(root)/'scene.yaml'
            source.write_text(yaml.safe_dump(scene(box())))
            store = SceneStore(load(source)[0], source=source)
            external = yaml.safe_dump(scene(box('agent')))
            source.write_text(external)
            result = command(store, 'clear')
            self.assertFalse(result['success'])
            self.assertIn('reload YAML', result['error'])
            self.assertEqual(store.revision, 1)
            self.assertEqual(source.read_text(), external)
            self.assertFalse(command(store, 'save')['success'])
            self.assertTrue(command(store, 'reload')['success'])
            self.assertEqual(store.document['obstacles'][0]['id'], 'agent')
            self.assertTrue(command(store, 'add', obstacle=box('human'))['success'])
            self.assertEqual([o['id'] for o in load(source)[0]['obstacles']], ['agent', 'human'])
            self.assertFalse(command(store, 'save', path='another.yaml')['success'])

    def test_save_failure_keeps_live_edit_and_retry_does_not_apply_it_twice(self):
        with tempfile.TemporaryDirectory() as root:
            source = Path(root)/'scene.yaml'
            source.write_text(yaml.safe_dump(scene(box())))
            store = SceneStore(load(source)[0], source=source)
            request = dict(requestId='edit', expectedEpoch=store.epoch, expectedRevision=1,
                           operation='add', obstacle=box('new'))
            with mock.patch('xgc2_scene_runtime.store.os.replace', side_effect=OSError('disk full')):
                result = store.command(request)
            self.assertFalse(result['success'])
            self.assertTrue(result['dirty'])
            self.assertIn('Live scene updated', result['error'])
            self.assertEqual(len(store.document['obstacles']), 2)
            self.assertEqual(len(load(source)[0]['obstacles']), 1)
            self.assertEqual(store.command(request), result)
            self.assertTrue(command(store, 'save')['success'])
            self.assertEqual(load(source)[0], store.document)
            self.assertFalse(store.envelope()['dirty'])

    def test_lost_reply_and_partial_apply_recover_without_reusing_revision(self):
        observed = {}
        fail = [False]
        def apply(doc, epoch, revision):
            self.assertNotIn(revision, observed)
            observed[revision] = copy.deepcopy(doc)
            if fail[0]:
                raise SceneError('reply lost after simulator changed')
        store = SceneStore(scene(box()), apply=apply)
        fail[0] = True
        self.assertFalse(command(store, 'clear')['success'])
        self.assertEqual(store.revision, 1)
        self.assertEqual(observed[2]['obstacles'], [])
        fail[0] = False
        self.assertTrue(command(store, 'resync')['success'])
        self.assertEqual(store.revision, 3)
        self.assertEqual(observed[3], store.document)
        self.assertFalse(store.envelope()['dirty'])
        self.assertEqual(store.undo_stack, [])

    def test_source_changed_during_startup_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as root:
            source = Path(root)/'scene.yaml'
            source.write_text(yaml.safe_dump(scene(box())))
            initial, sha = load(source)
            source.write_text('# external edit during startup\n'+source.read_text())
            store = SceneStore(initial, source=source, source_digest=sha)
            self.assertFalse(command(store, 'save')['success'])

    def test_motion_clock_and_saved_initial_definition_remain_separate(self):
        clock = [100.0]
        moving = box()
        moving['motion'] = {'type': 'constant_twist', 'linear': [2, 0, 0], 'angular': [0, 0, 0]}
        store = SceneStore(scene(moving), clock=lambda: clock[0])
        command(store, 'play')
        clock[0] += 3
        self.assertEqual(store.states()[0]['pose']['position'], [7, 2, 3])
        command(store, 'pause')
        clock[0] += 10
        self.assertEqual(store.states()[0]['pose']['position'], [7, 2, 3])
        self.assertEqual(store.states()[0]['linear'], [0, 0, 0])
        self.assertEqual(store.document['obstacles'][0]['pose']['position'], [1, 2, 3])
        self.assertFalse(store.envelope()['dirty'])
        command(store, 'reset')
        self.assertEqual(store.states()[0]['pose']['position'], [1, 2, 3])

    def test_motion_path_and_initial_pose_cannot_disagree(self):
        moving = box()
        moving['motion'] = {'type': 'ping_pong', 'point_a': [1, 2, 3], 'point_b': [4, 2, 3], 'speed': 1}
        accepted = document(scene(moving))['obstacles'][0]
        self.assertEqual(state(accepted, 0, False)['pose'], accepted['pose'])
        self.assertEqual(state(accepted, 2, True)['pose']['position'], [3, 2, 3])
        moving['pose']['position'] = [2, 2, 3]
        with self.assertRaises(SceneError):
            document(scene(moving))
        moving['motion'] = {'type': 'circle', 'center': [1, 2, 3], 'radius': 1, 'angular_speed': 1, 'phase': 0}
        accepted = document(scene(moving))['obstacles'][0]
        self.assertEqual(state(accepted, 0, False)['pose'], accepted['pose'])

    def test_full_snapshot_can_be_empty_but_bad_document_cannot_clear_it(self):
        store = SceneStore(scene(box()))
        self.assertFalse(command(store, 'replace', document={'obstacles': []})['success'])
        self.assertEqual(len(store.document['obstacles']), 1)
        self.assertTrue(command(store, 'clear')['success'])
        self.assertEqual(store.command({'operation': 'get'})['document']['obstacles'], [])

    def test_experiment_working_files_are_isolated_and_persist_edits(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source = root/'shared.yaml'
            source.write_text(yaml.safe_dump(scene(box('shared'))))
            original_source = source.read_bytes()
            a_file = root/'experiment-a'/'scene.yaml'
            b_file = root/'experiment-b'/'scene.yaml'
            a = SceneStore(load(source)[0], source=source, save_root=root, working_file=a_file)
            b = SceneStore(load(source)[0], source=source, save_root=root, working_file=b_file)

            self.assertTrue(command(a, 'add', obstacle=box('a-only'))['success'])
            self.assertTrue(command(b, 'add', obstacle=box('b-only'))['success'])
            self.assertEqual([item['id'] for item in load(source)[0]['obstacles']], ['shared'])
            self.assertEqual([item['id'] for item in load(a_file)[0]['obstacles']], ['shared', 'a-only'])
            self.assertEqual([item['id'] for item in load(b_file)[0]['obstacles']], ['shared', 'b-only'])

            b_bytes = b_file.read_bytes()
            self.assertTrue(command(a, 'undo')['success'])
            self.assertEqual([item['id'] for item in load(a_file)[0]['obstacles']], ['shared'])
            self.assertEqual(b_file.read_bytes(), b_bytes)
            self.assertTrue(command(a, 'redo')['success'])
            self.assertEqual([item['id'] for item in load(a_file)[0]['obstacles']], ['shared', 'a-only'])
            self.assertEqual(b_file.read_bytes(), b_bytes)

            self.assertTrue(command(a, 'save')['success'])
            self.assertEqual(source.read_bytes(), original_source)
            self.assertEqual(b_file.read_bytes(), b_bytes)
            self.assertTrue(command(a, 'reload')['success'])
            self.assertEqual([item['id'] for item in a.document['obstacles']], ['shared', 'a-only'])
            self.assertEqual(source.read_bytes(), original_source)
            self.assertEqual(b_file.read_bytes(), b_bytes)

            reopened_a = SceneStore(load(source)[0], source=source, save_root=root, working_file=a_file)
            reopened_b = SceneStore(load(source)[0], source=source, save_root=root, working_file=b_file)
            self.assertEqual(reopened_a.document, a.document)
            self.assertEqual(reopened_b.document, b.document)
            self.assertEqual(source.read_bytes(), original_source)

    def test_working_file_is_created_on_first_edit_before_apply(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source = root/'shared.yaml'
            source.write_text(yaml.safe_dump(scene(box('shared'))))
            original_source = source.read_bytes()
            working_file = root/'experiment'/'scene.yaml'
            apply_observations = []

            def apply(doc, epoch, revision):
                if revision > 1:
                    self.assertTrue(working_file.is_file())
                    apply_observations.append((copy.deepcopy(doc), load(working_file)[0]))

            store = SceneStore(load(source)[0], source=source, save_root=root,
                               working_file=working_file, apply=apply)
            self.assertFalse(working_file.exists())
            self.assertTrue(command(store, 'add', obstacle=box('experiment-only'))['success'])

            self.assertEqual(len(apply_observations), 1)
            applied_doc, copied_doc = apply_observations[0]
            self.assertEqual([item['id'] for item in applied_doc['obstacles']],
                             ['shared', 'experiment-only'])
            self.assertEqual([item['id'] for item in copied_doc['obstacles']], ['shared'])
            self.assertEqual([item['id'] for item in load(working_file)[0]['obstacles']],
                             ['shared', 'experiment-only'])
            self.assertEqual(source.read_bytes(), original_source)

    def test_first_working_copy_write_failure_does_not_apply_or_change_source(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source = root/'shared.yaml'
            source.write_text(yaml.safe_dump(scene(box('shared'))))
            original_source = source.read_bytes()
            working_file = root/'experiment'/'scene.yaml'
            applied = []
            store = SceneStore(load(source)[0], source=source, save_root=root,
                               working_file=working_file,
                               apply=lambda doc, epoch, revision: applied.append(copy.deepcopy(doc)))
            before = copy.deepcopy(store.document)
            before_revision = store.revision
            before_apply_count = len(applied)

            with mock.patch('xgc2_scene_runtime.store.os.replace', side_effect=OSError('disk full')):
                result = command(store, 'add', obstacle=box('experiment-only'))

            self.assertFalse(result['success'])
            self.assertIn('disk full', result['error'])
            self.assertEqual(store.document, before)
            self.assertEqual(store.revision, before_revision)
            self.assertEqual(len(applied), before_apply_count)
            self.assertFalse(working_file.exists())
            self.assertEqual(source.read_bytes(), original_source)

    def test_existing_working_file_wins_over_changed_source_on_reopen(self):
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            source = root/'shared.yaml'
            source.write_text(yaml.safe_dump(scene(box('shared'))))
            working_file = root/'experiment'/'scene.yaml'
            first = SceneStore(load(source)[0], source=source, save_root=root, working_file=working_file)
            self.assertTrue(command(first, 'add', obstacle=box('experiment-only'))['success'])
            working_document, _ = load(working_file)

            changed_source = yaml.safe_dump(scene(box('asset-changed-later')))
            source.write_text(changed_source)
            changed_source_bytes = source.read_bytes()
            reopened = SceneStore(load(source)[0], source=source, save_root=root, working_file=working_file)

            self.assertEqual(reopened.document, working_document)
            self.assertEqual([item['id'] for item in reopened.document['obstacles']],
                             ['shared', 'experiment-only'])
            self.assertEqual(source.read_bytes(), changed_source_bytes)
            self.assertTrue(command(reopened, 'reload')['success'])
            self.assertEqual(reopened.document, working_document)
            self.assertEqual(source.read_bytes(), changed_source_bytes)


if __name__ == '__main__':
    unittest.main()
