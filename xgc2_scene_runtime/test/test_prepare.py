import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml

from xgc2_scene_runtime.document import SceneError
from xgc2_scene_runtime.prepare import prepare

WORLD = b'<sdf version="1.6"><world name="fixture"><physics type="ode" name="physics"><max_step_size>0.001</max_step_size></physics><model name="authored"><static>true</static><link name="body"><collision name="collision"><geometry><box><size>1 2 3</size></box></geometry></collision></link></model></world></sdf>'
DOC = {'schema': 'xgc2.scene.v1', 'id': 'fixture', 'frame': 'world', 'obstacles': []}


class PrepareTest(unittest.TestCase):
    def fixture(self, directory, mode='native'):
        root = Path(directory)
        asset = root/'asset'; asset.mkdir()
        out = root/'run'; out.mkdir()
        (asset/'source.world').write_bytes(WORLD)
        (asset/'scene.yaml').write_text(yaml.safe_dump(DOC))
        def attachment(name):
            return {'file': name, 'sha256': hashlib.sha256((asset/name).read_bytes()).hexdigest()}
        manifest = {'schemaVersion': 1, 'kind': 'xgc.scene-replay-asset.v1', 'revisionId': 'revision1',
                    'simulators': {'gazebo': {'geometry': mode}}, 'gazeboWorld': attachment('source.world'),
                    'sceneDocument': attachment('scene.yaml')}
        (asset/'manifest.json').write_text(json.dumps(manifest))
        request = {'schema': 'xgc2.simulation.prepare.v1', 'configuration_revision': 4,
                   'asset_directory': str(asset), 'expected_revision_id': 'revision1',
                   'resource_root': str(root), 'socket_path': str(out/'world.sock'), 'target_id': 'fixture',
                   'output_grant': {'directory': str(out), 'max_bytes': 1048576}}
        return request, asset, out, manifest

    def test_native_world_keeps_exact_geometry_and_owns_only_its_granted_output(self):
        with tempfile.TemporaryDirectory() as directory:
            request, asset, out, manifest = self.fixture(directory)
            result = prepare(request)
            self.assertTrue(result['frozen'])
            self.assertFalse(result['projectGeometry'])
            self.assertEqual((asset/'source.world').read_bytes(), WORLD)
            source = ET.fromstring(WORLD)
            prepared = ET.parse(result['worldFile']).getroot()
            self.assertEqual(ET.tostring(source.find('world/model')), ET.tostring(prepared.find('world/model')))
            plugin = prepared.find('world/plugin')
            self.assertEqual(plugin.get('filename'), 'libxgc2_simulation_world.so')
            self.assertEqual(plugin.findtext('target_id'), 'fixture')
            self.assertEqual(plugin.findtext('configuration_revision'), '4')
            self.assertEqual(prepared.findtext('world/physics/max_step_size'), '0.001')
            self.assertEqual(result['configuration_revision'], 4)
            self.assertEqual(result['writes'], [str(out/'prepared-world.world')])

    def test_document_editability_and_media_attachments_are_provider_owned(self):
        with tempfile.TemporaryDirectory() as directory:
            request, asset, out, manifest = self.fixture(directory, 'document')
            manifest['simulators']['xsim'] = {'geometry': 'document'}
            for key, filename in [('media', 'camera.png'), ('cameraInfo', 'calibration.yaml'), ('extrinsic', 'extrinsic.yaml')]:
                (asset/filename).write_bytes(b'opaque-owning-product-data')
                manifest[key] = {'file': filename, 'sha256': hashlib.sha256((asset/filename).read_bytes()).hexdigest()}
            (asset/'manifest.json').write_text(json.dumps(manifest))
            request.update(editable=True, parameters={'overrideWorldPhysicsTiming': True, 'maxStepSize': .002,
                                                       'realTimeUpdateRate': 0})
            result = prepare(request)
            self.assertFalse(result['frozen'])
            self.assertEqual(result['workingFile'], str(out/'scene-document.yaml'))
            self.assertEqual(yaml.safe_load(Path(result['sceneFile']).read_text()), DOC)
            self.assertEqual(result['mediaFile'], str(asset/'camera.png'))
            self.assertEqual(result['cameraInfoFile'], str(asset/'calibration.yaml'))
            self.assertEqual(ET.parse(result['worldFile']).findtext('world/physics/max_step_size'), '0.002')

    def test_wrong_revision_bad_hash_unknown_physics_and_symlink_escape_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            request, asset, out, manifest = self.fixture(directory)
            for values in [{'expected_revision_id': 'wrong'}, {'parameters': {'typo': 1}},
                           {'parameters': {'maxStepSize': True}}, {'configuration_revision': 4.0},
                           {'output_grant': {'directory': str(out), 'max_bytes': 1}}]:
                with self.subTest(values=values), self.assertRaises(SceneError):
                    prepare(dict(request, **values))
            manifest['gazeboWorld']['sha256'] = '0'*64
            (asset/'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaises(SceneError): prepare(request)
            self.assertFalse((out/'prepared-world.world').exists())

    def test_random_generator_output_is_bounded_before_world_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            request, asset, out, manifest = self.fixture(directory, 'document')
            source = {'schema': 'xgc2.scene-source.v1', 'format': 'geometry', 'mode': 'random',
                      'generator': {'command': [sys.executable, '-c', 'import sys; sys.stdout.write("x"*2000000)'],
                                    'parameters': {}}}
            (asset/'scene.yaml').write_text(yaml.safe_dump(source))
            manifest['sceneDocument']['sha256'] = hashlib.sha256((asset/'scene.yaml').read_bytes()).hexdigest()
            manifest['obstacleInput'] = {'mode': 'random', 'format': 'geometry'}
            (asset/'manifest.json').write_text(json.dumps(manifest))
            request['run_id'] = 'run1'
            with self.assertRaisesRegex(SceneError, 'byte limit'):
                prepare(request)
            self.assertEqual(list(out.iterdir()), [])

    def test_explicit_chassis_and_component_rosters_are_not_inferred(self):
        with tempfile.TemporaryDirectory() as directory:
            request, asset, out, manifest = self.fixture(directory)
            request['parameters'] = {'chassis_robot_ids': ['ugv1', 'ugv2'], 'required_components': ['vrpn']}
            result = prepare(request)
            plugin = ET.parse(result['worldFile']).find('world/plugin')
            self.assertEqual([item.text for item in plugin.findall('chassis_robot_id')], ['ugv1', 'ugv2'])
            self.assertEqual([item.text for item in plugin.findall('required_component')], ['vrpn'])
            for name in ('chassis_robot_ids', 'required_components'):
                for roster in [['duplicate', 'duplicate'], ['机器人'], ['../escape'], list(map(str, range(17))), 'ugv1']:
                    with self.subTest(name=name, roster=roster), self.assertRaises(SceneError):
                        prepare(dict(request, parameters={name: roster}))

    def test_asset_attachment_symlink_cannot_escape_its_asset_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            request, asset, out, manifest = self.fixture(directory)
            outside = Path(directory)/'outside.world'; outside.write_bytes(WORLD)
            (asset/'source.world').unlink(); (asset/'source.world').symlink_to(outside)
            with self.assertRaisesRegex(SceneError, 'outside the granted root'):
                prepare(request)


if __name__ == '__main__':
    unittest.main()
