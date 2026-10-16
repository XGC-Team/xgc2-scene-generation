from pathlib import Path
import tempfile
import unittest

import yaml

from xgc2_scene_runtime.document import SceneError, check_obstacle_set, document
from xgc2_scene_runtime import motion
from xgc2_scene_runtime.motion import state
from xgc2_scene_runtime.store import DocumentYaml, SceneStore, dump_yaml, load


def box(oid='box'):
    return {'id': oid, 'pose': {'position': [1, 2, 3]}, 'parts': [
        {'id': 'body', 'geometry': {'type': 'box', 'size': [1, 2, 3]}}]}


def command(store, operation, **values):
    return store.command(dict(requestId='r{}'.format(len(store.requests)), expectedEpoch=store.epoch,
                              expectedRevision=store.revision, operation=operation, **values))


def wide_parts():
    return [{'id': 'p{}'.format(index), 'geometry': {'type': 'sphere', 'radius': 0.1}} for index in range(128)]


def authored():
    return {'schema': 'xgc2.scene.v1', 'id': 'yard', 'frame': 'world',
            'models': {
                'barrier': {'resource': {'format': 'sdformat-model', 'file': 'models/barrier/model.sdf'}},
                'wide': {'parts': wide_parts()}},
            'ground': {'mode': 'model', 'model': 'barrier', 'pose': {'position': [0, 0, -0.2]}},
            'obstacles': [{
                'id': 'gate', 'name': 'gate', 'model': 'barrier',
                'pose': {'position': [2, 0, 0.5]},
                'motion': {'type': 'waypoints', 'velocity': 2.0, 'orientation': False, 'angular_velocity': 0.8,
                           'loop': False, 'sin': True,
                           'waypoints': [[2, 0, 0.5], [6, 0, 0.5], [6, 4, 0.5]]}}]}


class SceneSourceTest(unittest.TestCase):
    def test_omitted_ground_stays_the_default_plane_and_scale_is_rejected(self):
        normalized = document({'schema': 'xgc2.scene.v1', 'id': 'plain', 'frame': 'world', 'obstacles': []})
        self.assertNotIn('ground', normalized)
        self.assertNotIn('models', normalized)
        explicit = document({'schema': 'xgc2.scene.v1', 'id': 'plain', 'frame': 'world',
                             'ground': {'mode': 'default_plane'}, 'obstacles': []})
        self.assertEqual(explicit['ground'], {'mode': 'default_plane', 'z': 0.0})
        raised = dict(explicit, ground={'mode': 'default_plane', 'z': 1.5})
        self.assertEqual(document(raised)['ground']['z'], 1.5)
        self.assertEqual(document(dict(explicit, ground={'mode': 'none'}))['ground'], {'mode': 'none'})
        with self.assertRaises(SceneError):
            document(dict(normalized, terrain={'mode': 'mesh'}))
        scaled = box()
        scaled['scale'] = [2, 2, 2]
        with self.assertRaises(SceneError):
            document({'schema': 'xgc2.scene.v1', 'id': 'plain', 'frame': 'world', 'obstacles': [scaled]})

    def test_model_reference_is_not_rewritten_as_inline_parts(self):
        normalized = document(authored())
        gate = normalized['obstacles'][0]
        self.assertEqual(gate['model'], 'barrier')
        self.assertNotIn('parts', gate)
        self.assertEqual(normalized['models']['barrier']['resource'],
                         {'format': 'sdformat-model', 'file': 'models/barrier/model.sdf'})
        self.assertNotIn('parts', normalized['models']['barrier'])
        self.assertEqual(normalized['ground']['model'], 'barrier')
        both = box()
        both['model'] = 'barrier'
        with self.assertRaises(SceneError) as mixed_obstacle:
            document({'schema': 'xgc2.scene.v1', 'id': 'plain', 'frame': 'map', 'models': authored()['models'],
                      'obstacles': [both]})
        self.assertIn('inline parts or a model reference', str(mixed_obstacle.exception))
        with self.assertRaises(SceneError) as mixed_model:
            document({'schema': 'xgc2.scene.v1', 'id': 'plain', 'frame': 'map', 'obstacles': [], 'models': {
                'mixed': {'parts': wide_parts()[:1], 'resource': {'format': 'mesh', 'file': 'meshes/a.obj'}}}})
        self.assertIn('analytic parts or a resource', str(mixed_model.exception))

    def test_model_part_budget_uses_the_referenced_model(self):
        defined = {'wide': {'parts': wide_parts()},
                   'file': {'resource': {'format': 'gltf', 'file': 'models/hall/hall.gltf'}}}
        with self.assertRaises(SceneError) as raised:
            document({'schema': 'xgc2.scene.v1', 'id': 'budget', 'frame': 'world', 'models': defined,
                      'obstacles': [{'id': 'w{}'.format(index), 'model': 'wide'} for index in range(33)]})
        self.assertIn('4096', str(raised.exception))
        full = document({'schema': 'xgc2.scene.v1', 'id': 'budget', 'frame': 'world', 'models': defined,
                         'obstacles': [{'id': 'w{}'.format(index), 'model': 'wide'} for index in range(32)]})
        self.assertNotIn('parts', full['obstacles'][0])
        self.assertEqual(check_obstacle_set(full['obstacles'], full['models']), full['obstacles'])
        added = command(SceneStore(full), 'add', obstacle={'id': 'hall', 'model': 'file'})
        self.assertTrue(added['success'], added)
        self.assertNotIn('parts', added['document']['obstacles'][-1])
        self.assertEqual(added['document']['models']['file']['resource']['file'], 'models/hall/hall.gltf')

    def test_save_edit_undo_and_reopen_keep_model_resource_and_ground(self):
        with tempfile.TemporaryDirectory() as root:
            source = Path(root)/'scene.yaml'
            source.write_text(yaml.safe_dump(document(authored())))
            store = SceneStore(load(source)[0], source=source)
            models, owned = store.document['models'], store.document['ground']
            self.assertTrue(command(store, 'add', obstacle=box('extra'))['success'])
            self.assertIs(store.document['models'], models)
            self.assertIs(store.document['ground'], owned)
            self.assertEqual(store.document['obstacles'][0]['model'], 'barrier')
            self.assertNotIn('parts', store.document['obstacles'][0])
            self.assertTrue(command(store, 'delete', id='extra')['success'])
            self.assertTrue(command(store, 'undo')['success'])
            self.assertEqual([item['id'] for item in store.document['obstacles']], ['gate', 'extra'])
            reopened = SceneStore(load(source)[0], source=source)
            self.assertEqual(reopened.document['models']['barrier']['resource']['file'], 'models/barrier/model.sdf')
            self.assertEqual(reopened.document['ground'], owned)
            self.assertNotIn('parts', reopened.document['obstacles'][0])
            self.assertEqual(reopened.document['obstacles'][0]['motion']['sin'], True)
            self.assertEqual(reopened.document['obstacles'][0]['motion']['waypoints'],
                             [[2.0, 0.0, 0.5], [6.0, 0.0, 0.5], [6.0, 4.0, 0.5]])
            self.assertEqual(DocumentYaml().dump(store.document), dump_yaml(store.document))

    def test_waypoints_are_stored_and_runtime_state_rejects_them(self):
        body = {'id': 'mover', 'pose': {'position': [0, 0, 0]}, 'parts': [
            {'id': 'body', 'geometry': {'type': 'box', 'size': [1, 1, 1]}}],
            'motion': {'type': 'waypoints', 'velocity': 2, 'orientation': False,
                       'waypoints': [[0, 0, 0], [4, 0, 0], [4, 4, 0]]}}
        normalized = document({'schema': 'xgc2.scene.v1', 'id': 'path', 'frame': 'world', 'obstacles': [body]})
        spec = normalized['obstacles'][0]['motion']
        self.assertEqual(list(spec), ['type', 'velocity', 'orientation', 'angular_velocity', 'loop', 'sin', 'waypoints'])
        self.assertEqual(spec['velocity'], 2.0)
        self.assertFalse(spec['orientation'])
        self.assertEqual(spec['angular_velocity'], 0.8)
        self.assertFalse(spec['loop'])
        self.assertFalse(spec['sin'])
        self.assertEqual(spec['waypoints'][2], [4.0, 4.0, 0.0])
        for playing in (True, False):
            with self.assertRaises(SceneError) as rejected:
                state(normalized['obstacles'][0], 1.0, playing)
            self.assertIn('unsupported', str(rejected.exception))
        sine = dict(body)
        sine['motion'] = dict(body['motion'], sin=True, loop=True, orientation=True, angular_velocity=0.4)
        fitted = document({'schema': 'xgc2.scene.v1', 'id': 'path', 'frame': 'world', 'obstacles': [sine]})
        kept = fitted['obstacles'][0]['motion']
        self.assertTrue(kept['sin'])
        self.assertTrue(kept['loop'])
        self.assertTrue(kept['orientation'])
        self.assertEqual(kept['angular_velocity'], 0.4)
        self.assertEqual(kept['waypoints'], spec['waypoints'])
        with self.assertRaises(SceneError) as sine_rejected:
            state(fitted['obstacles'][0], 3.0, True)
        self.assertIn('unsupported', str(sine_rejected.exception))
        self.assertEqual(fitted['obstacles'][0]['motion']['waypoints'], kept['waypoints'])
        self.assertFalse(hasattr(motion, 'waypoint_state'))
        with self.assertRaises(SceneError) as runtime:
            SceneStore(fitted).states()
        self.assertIn('unsupported', str(runtime.exception))
        shorthand = dict(body)
        shorthand['motion'] = {'type': 'waypoints', 'points': [[0, 0, 0], [1, 0, 0]], 'speed': 1, 'align_yaw': True}
        with self.assertRaises(SceneError):
            document({'schema': 'xgc2.scene.v1', 'id': 'path', 'frame': 'world', 'obstacles': [shorthand]})

    def test_waypoint_pose_and_first_point_stay_five_metres_apart(self):
        body = {'id': 'mover', 'pose': {'position': [0, 0, 0], 'orientation': [0, 0, 0, 1]},
                'parts': [{'id': 'body', 'geometry': {'type': 'box', 'size': [1, 1, 1]}}],
                'motion': {'type': 'waypoints', 'waypoints': [[5, 0, 0], [6, 0, 0]]}}
        scene = {'schema': 'xgc2.scene.v1', 'id': 'path', 'frame': 'world', 'obstacles': [body]}
        stored = document(scene)
        obstacle = stored['obstacles'][0]
        self.assertEqual(obstacle['pose']['position'], [0.0, 0.0, 0.0])
        self.assertEqual(obstacle['motion']['waypoints'][0], [5.0, 0.0, 0.0])
        self.assertEqual(len(obstacle['motion']['waypoints']), 2)
        with tempfile.TemporaryDirectory() as root:
            source = Path(root)/'scene.yaml'
            source.write_text(yaml.safe_dump(stored))
            loaded = load(source)[0]
            store = SceneStore(loaded, source=source)
            self.assertTrue(command(store, 'save')['success'])
            reopened = load(source)[0]['obstacles'][0]
        self.assertEqual(reopened['pose']['position'], [0.0, 0.0, 0.0])
        self.assertEqual(reopened['motion']['waypoints'], [[5.0, 0.0, 0.0], [6.0, 0.0, 0.0]])
        with self.assertRaises(SceneError) as rejected:
            state(reopened, 1.0, True)
        self.assertIn('unsupported', str(rejected.exception))
        shuttle = box()
        shuttle['motion'] = {'type': 'ping_pong', 'point_a': [0, 0, 0], 'point_b': [1, 0, 0], 'speed': 1}
        with self.assertRaises(SceneError):
            document({'schema': 'xgc2.scene.v1', 'id': 'motion', 'frame': 'world', 'obstacles': [shuttle]})

    def test_analytic_motions_keep_their_numeric_states(self):
        hold = document({'schema': 'xgc2.scene.v1', 'id': 'motion', 'frame': 'world', 'obstacles': [box()]})
        held = state(hold['obstacles'][0], 5, True)
        self.assertEqual(held['pose']['position'], [1.0, 2.0, 3.0])
        self.assertEqual(held['linear'], [0.0, 0.0, 0.0])
        self.assertEqual(held['angular'], [0.0, 0.0, 0.0])
        twist = box()
        twist['motion'] = {'type': 'constant_twist', 'linear': [2, 0, 0], 'angular': [0, 0, 0]}
        twisting = document({'schema': 'xgc2.scene.v1', 'id': 'motion', 'frame': 'world', 'obstacles': [twist]})
        moved = state(twisting['obstacles'][0], 3, True)
        self.assertEqual(moved['pose']['position'], [7.0, 2.0, 3.0])
        self.assertEqual(moved['linear'], [2.0, 0.0, 0.0])
        paused = state(twisting['obstacles'][0], 3, False)
        self.assertEqual(paused['pose']['position'], [7.0, 2.0, 3.0])
        self.assertEqual(paused['linear'], [0.0, 0.0, 0.0])
        shuttle = box()
        shuttle['motion'] = {'type': 'ping_pong', 'point_a': [1, 2, 3], 'point_b': [4, 2, 3], 'speed': 1}
        shuttling = document({'schema': 'xgc2.scene.v1', 'id': 'motion', 'frame': 'world', 'obstacles': [shuttle]})
        self.assertEqual(state(shuttling['obstacles'][0], 2, True)['pose']['position'], [3.0, 2.0, 3.0])
        self.assertEqual(state(shuttling['obstacles'][0], 2, True)['linear'], [1.0, 0.0, 0.0])
        orbit = box()
        orbit['pose'] = {'position': [2, 2, 3]}
        orbit['motion'] = {'type': 'circle', 'center': [1, 2, 3], 'radius': 1, 'angular_speed': 1, 'phase': 0}
        orbiting = document({'schema': 'xgc2.scene.v1', 'id': 'motion', 'frame': 'world', 'obstacles': [orbit]})
        circled = state(orbiting['obstacles'][0], 0, True)
        self.assertEqual(circled['pose']['position'], [2.0, 2.0, 3.0])
        self.assertEqual(circled['linear'], [0.0, 1.0, 0.0])

    def test_convex_triangles_stay_a_flat_index_list(self):
        tetra = {'type': 'convex', 'vertices': [[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]],
                 'triangles': [0, 1, 2, 0, 3, 1, 0, 2, 3, 1, 3, 2]}
        obstacle = {'id': 'rock', 'parts': [{'id': 'body', 'geometry': tetra}]}
        stored = document({'schema': 'xgc2.scene.v1', 'id': 'mesh', 'frame': 'world', 'obstacles': [obstacle]})
        triangles = stored['obstacles'][0]['parts'][0]['geometry']['triangles']
        self.assertEqual(len(triangles), 12)
        self.assertTrue(all(type(index) is int for index in triangles))
        self.assertNotIsInstance(triangles[0], list)


if __name__ == '__main__':
    unittest.main()
