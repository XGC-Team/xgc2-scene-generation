"""Offline producer tests: no ROS, HTTP, third-party packages or subprocesses."""

import copy
import hashlib
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest import mock

from xgc2_scene_runtime import scene_sources as sources


def pcd(body, *, encoding='ascii', fields='x y z', sizes='4 4 4',
        types='F F F', counts='1 1 1', count=2):
    header = ('# fixture\nVERSION .7\nFIELDS {}\nSIZE {}\nTYPE {}\nCOUNT {}\n'
              'WIDTH {}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {}\nDATA {}\n').format(
                  fields, sizes, types, counts, count, count, encoding)
    return header.encode('ascii') + (body.encode('ascii') if isinstance(body, str) else body)


def ply(body, *, encoding='ascii', properties='property float x\nproperty float y\nproperty float z\n',
        count=2, suffix=''):
    header = 'ply\nformat {} 1.0\ncomment fixture\nelement vertex {}\n{}{}end_header\n'.format(
        encoding, count, properties, suffix)
    return header.encode('ascii') + (body.encode('ascii') if isinstance(body, str) else body)


def unpack(data):
    return list(struct.iter_unpack('<fff', data))


class SceneSourcesTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def file(self, name, data):
        path = self.root / name
        path.write_bytes(data.encode('ascii') if isinstance(data, str) else data)
        return path

    def imported(self, data, format_name='pcd', source='pointcloud', **parameters):
        path = self.file('source.' + format_name, data)
        before = path.read_bytes()
        contents = sorted(item.name for item in self.root.iterdir())
        result = sources.generate_scene('Imported map', source,
                                        dict(format=format_name, resolution=0.1, representation='surface', **parameters),
                                        input_path=path)
        self.assertEqual(path.read_bytes(), before)
        self.assertEqual(sorted(item.name for item in self.root.iterdir()), contents)
        self.assertEqual(result['scene']['ground'], {'mode': 'none'})
        self.assertEqual(result['scene']['obstacles'], [])
        self.assertEqual(result['recipe']['geometry_status'], 'not_reconstructed')
        self.assertEqual(result['recipe']['point_cloud_role'], 'sensing_source')
        original = result['recipe']['input']
        self.assertEqual(result['resources'][original['file']], before)
        self.assertEqual(original['sha256'], hashlib.sha256(before).hexdigest())
        self.assertEqual(result['resources']['reference.xyz'], result['point_cloud'])
        self.assertNotIn(str(self.root), json.dumps(result['recipe']))
        return result

    def test_primitive_seed_layout_and_parametric_holes(self):
        parameters = dict(seed=37, count=15, ring_count=5)
        one = sources.generate_scene('Column forest', 'primitive-forest', parameters)
        self.assertEqual(one, sources.generate_scene('Column forest', 'primitive-forest', parameters))
        other = sources.generate_scene('Column forest', 'primitive-forest', dict(parameters, seed=38))
        self.assertNotEqual(one['scene'], other['scene'])
        self.assertEqual(one['scene']['schema'], 'xgc2.scene.v1')
        self.assertEqual(one['scene']['id'], 'Column-forest')
        self.assertEqual(len(one['scene']['obstacles']), 20)
        self.assertTrue(one['resources']['reference.xyz'])
        self.assertIsNone(one['point_cloud'])
        self.assertEqual(one['recipe']['geometry_status'], 'parameterized')
        self.assertEqual(one['recipe']['point_cloud_role'], 'reference_surface')
        self.assertEqual(len(one['resources']['reference.xyz']), one['recipe']['point_count'] * 12)
        positions = [item['pose']['position'] for item in one['scene']['obstacles']]
        for index, position in enumerate(positions):
            self.assertTrue(-20 <= position[0] <= 20)
            self.assertTrue(-20 <= position[1] <= 20)
            for other_position in positions[index + 1:]:
                self.assertGreaterEqual(math.hypot(position[0] - other_position[0], position[1] - other_position[1]), 1.5)
        for item in one['scene']['obstacles'][15:]:
            geometry = item['parts'][0]['geometry']
            self.assertEqual(geometry, dict(type='ring', radii=[1.2, 0.9], tubeRadius=0.12))
            self.assertGreater(min(geometry['radii']), geometry['tubeRadius'])
            self.assertAlmostEqual(sum(value**2 for value in item['pose']['orientation']), 1)
            # Upright gate: quaternion's transformed local Z is horizontal.
            qx, qy, _, _ = item['pose']['orientation']
            self.assertAlmostEqual(1 - 2 * (qx*qx + qy*qy), 0)

    def test_primitive_reference_sampling_preserves_authority_and_ring_hole(self):
        parameters = dict(seed=7, count=0, ring_count=1, samples_per_m2=100)
        result = sources.generate_scene('Gate', 'primitive-forest', parameters)
        scene = result['scene']
        position = scene['obstacles'][0]['pose']['position']
        orientation = scene['obstacles'][0]['pose']['orientation']
        qx, qy, qz, qw = orientation
        # Inverse rotate samples into the unchanged local XY ring plane.
        inverse = ((1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy + qz*qw), 2 * (qx*qz - qy*qw)),
                   (2 * (qx*qy - qz*qw), 1 - 2 * (qx*qx + qz*qz), 2 * (qy*qz + qx*qw)),
                   (2 * (qx*qz + qy*qw), 2 * (qy*qz - qx*qw), 1 - 2 * (qx*qx + qy*qy)))
        for point in unpack(result['resources']['reference.xyz']):
            local = [sum(inverse[i][j] * (point[j] - position[j]) for j in range(3)) for i in range(3)]
            self.assertGreater(math.hypot(local[0], local[1]), 0.77)
            self.assertLessEqual(abs(local[2]), .121)
        denser = sources.generate_scene('Gate', 'primitive-forest', dict(parameters, samples_per_m2=200))
        self.assertEqual(scene, denser['scene'])
        self.assertGreater(denser['recipe']['point_count'], result['recipe']['point_count'])
        self.assertIn('tessellation', result['recipe'])

    def test_catalog_defaults_are_the_validation_and_ui_authority(self):
        self.assertEqual({item['id'] for item in sources.SOURCE_CATALOG},
                         {'primitive-forest', 'tree-forest', 'pointcloud', 'mockamap'})
        for descriptor in sources.SOURCE_CATALOG:
            defaults = {item['name']: item['default'] for item in descriptor['parameters']}
            self.assertEqual(defaults, sources.DEFAULT_PARAMETERS[descriptor['id']])
            self.assertEqual(len(defaults), len(descriptor['parameters']))
            if descriptor['id'] in ('pointcloud', 'mockamap'):
                path = self.file('map.pcd', pcd('1 2 3\n4 5 6\n'))
                result = sources.generate_scene('defaults', descriptor['id'], defaults, input_path=path)
            else:
                result = sources.generate_scene('defaults', descriptor['id'], defaults)
            self.assertEqual(result['recipe']['parameters'], defaults)

    def test_numeric_validation_and_bounded_layout_fail_explicitly(self):
        for parameters in [dict(seed=True), dict(seed=1.0), dict(seed=-1), dict(seed=2**53),
                           dict(count=2.0), dict(count=True), dict(count=513), dict(count=512, ring_count=1),
                           dict(radius_min=2, radius_max=1), dict(x_min=2, x_max=2),
                           dict(tube_radius=0.9), dict(height_min=float('nan')),
                           dict(min_distance=float('inf')), dict(seed=10**1000),
                           dict(command='echo bad'), dict(frame='map')]:
            with self.subTest(parameters=parameters), self.assertRaises(ValueError):
                sources.generate_scene('bad', 'primitive-forest', parameters)
        with self.assertRaisesRegex(ValueError, 'bounded layout'):
            sources.generate_scene('packed', 'primitive-forest',
                                   dict(count=10, ring_count=0, x_min=0, x_max=1, y_min=0, y_max=1, min_distance=2))
        with self.assertRaises(ValueError):
            sources.generate_scene('../outside', 'primitive-forest', {})
        with self.assertRaises(ValueError):
            sources.generate_scene('map', 'unsupported', {})
        with self.assertRaises(ValueError):
            sources.generate_scene('map', 'primitive-forest', [])

    def test_zero_count_scene_is_bounded_and_does_not_invent_ground(self):
        result = sources.generate_scene('empty', 'primitive-forest', dict(count=0, ring_count=0))
        self.assertEqual(result['scene']['obstacles'], [])
        self.assertEqual(result['scene']['ground'], {'mode': 'none'})
        self.assertEqual(result['resources'], {})

    def test_tree_instances_reuse_scaled_resources_and_keep_original_input(self):
        raw = b'# original tree asset\nv 0 0 0\nv 1 0 0\nv 0 1 2\nf 1 2 3\n'
        path = self.file('tree.obj', raw)
        parameters = dict(seed=5, count=20, samples_per_m2=10)
        before = copy.deepcopy(parameters)
        result = sources.generate_scene('Trees', 'tree-forest', parameters, tree_models=[path, path])
        self.assertEqual(parameters, before)
        self.assertEqual(path.read_bytes(), raw)
        self.assertEqual(list(self.root.iterdir()), [path])
        self.assertEqual(result, sources.generate_scene('Trees', 'tree-forest', parameters, tree_models=[path, path]))
        self.assertEqual(result['recipe']['tree_template'], 'external-obj')
        self.assertEqual(len(result['recipe']['tree_inputs']), 1)
        self.assertEqual(result['resources']['originals/tree-0.obj'], raw)
        self.assertIsNone(result['point_cloud'])
        self.assertEqual(result['recipe']['point_cloud_role'], 'reference_surface')
        self.assertNotIn(str(path), json.dumps(result['recipe']))
        self.assertEqual(len(result['scene']['obstacles']), 20)
        self.assertLessEqual(len(result['scene']['models']), 3)
        self.assertGreaterEqual(len(result['scene']['models']), 2)
        references = [item['model'] for item in result['scene']['obstacles']]
        self.assertLess(len(set(references)), len(references))
        for item in result['scene']['obstacles']:
            self.assertNotIn('parts', item)
            self.assertNotIn('scale', item)
            file = result['scene']['models'][item['model']]['resource']['file']
            self.assertTrue(result['resources'][file].startswith(b'# Platform geometry-only OBJ; uniform scale baked'))
        expected_points = result['recipe']['point_count']
        self.assertEqual(len(result['resources']['reference.xyz']), expected_points * 12)
        denser = sources.generate_scene('Trees', 'tree-forest', dict(parameters, samples_per_m2=30), tree_models=[path])
        self.assertEqual(result['scene'], denser['scene'])
        self.assertGreater(denser['recipe']['point_count'], expected_points)

    def test_tree_surface_sampling_uses_triangle_area_and_uniform_barycentrics(self):
        # Areas 1 and 9, in separate horizontal planes so counts are measurable.
        path = self.file('two-areas.obj', 'v 0 0 1\nv 1 0 1\nv 0 2 1\n'
                         'v 0 0 2\nv 3 0 2\nv 0 6 2\nf 1 2 3\nf 4 5 6\n')
        result = sources.generate_scene('sample', 'tree-forest',
                                        dict(seed=19, count=1, scale_min=1, scale_max=1, samples_per_m2=1000),
                                        tree_models=[path])
        points = unpack(result['resources']['reference.xyz'])
        self.assertEqual(len(points), 10000)
        small = [point for point in points if point[2] == 1]
        large = [point for point in points if point[2] == 2]
        self.assertAlmostEqual(len(small) / len(points), 0.1, delta=0.015)
        self.assertEqual(len(small) + len(large), len(points))
        obstacle = result['scene']['obstacles'][0]
        x, y, _ = obstacle['pose']['position']
        yaw = 2 * math.atan2(obstacle['pose']['orientation'][2], obstacle['pose']['orientation'][3])
        local_x = [(p[0] - x) * math.cos(yaw) + (p[1] - y) * math.sin(yaw) for p in large]
        local_y = [-(p[0] - x) * math.sin(yaw) + (p[1] - y) * math.cos(yaw) for p in large]
        self.assertAlmostEqual(sum(local_x) / len(large), 1, delta=0.04)
        self.assertAlmostEqual(sum(local_y) / len(large), 2, delta=0.07)

    def test_platform_default_tree_is_explicit_and_has_branches(self):
        result = sources.generate_scene('trees', 'tree-forest', dict(seed=1, count=3))
        self.assertEqual(result['recipe']['tree_template'], 'platform-branching-template-v1')
        self.assertTrue(result['resources']['reference.xyz'])
        for model in result['scene']['models'].values():
            geometry = result['resources'][model['resource']['file']].decode('ascii')
            self.assertGreater(geometry.count('\nv '), 12)
            self.assertGreater(geometry.count('\nf '), 12)

    def test_obj_convex_polygons_negative_indices_and_rejections(self):
        accepted = self.file('quad.obj', 'v 0 0 0\nv 2 0 0\nv 2 2 0\nv 0 2 0\nf -4 -3 -2 -1\n')
        result = sources.generate_scene('quad', 'tree-forest', dict(count=1, scale_min=1, scale_max=1), tree_models=[accepted])
        model = next(iter(result['scene']['models'].values()))
        self.assertEqual(result['resources'][model['resource']['file']].count(b'\nf '), 2)
        bad = [b'v nan 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n',
               b'v 0 0 0\nv 1 0 0\nv 2 0 0\nf 1 2 3\n',
               b'v 0 0 0\nv 1 0 0\nv 0 1 0\nf 0 2 3\n',
               b'v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 8\n',
               b'v 0 0 0\nv 2 0 0\nv 1 .5 0\nv 2 2 0\nv 0 2 0\nf 1 2 3 4 5\n',
               b'v 0 0 0\nv 1 0 0\nv 1 1 1\nv 0 1 0\nf 1 2 3 4\n']
        for index, raw in enumerate(bad):
            path = self.file('bad-{}.obj'.format(index), raw)
            with self.subTest(index=index), self.assertRaises(ValueError):
                sources.generate_scene('bad', 'tree-forest', dict(count=1), tree_models=[path])
            self.assertEqual(path.read_bytes(), raw)

    def test_tree_resource_and_sampling_requests_are_bounded(self):
        for parameters in [dict(count=513), dict(scale_variants=9), dict(samples_per_m2=1e9),
                           dict(scale_min=3, scale_max=2), dict(scale_min=float('inf'))]:
            with self.subTest(parameters=parameters), self.assertRaises(ValueError):
                sources.generate_scene('bad', 'tree-forest', parameters)
        with mock.patch.object(sources, 'MAX_POINTS', 1), self.assertRaisesRegex(ValueError, 'sampling exceeds'):
            sources.generate_scene('too-dense', 'tree-forest', dict(count=1))
        with self.assertRaises(ValueError):
            sources.generate_scene('bad', 'tree-forest', {}, tree_models='arbitrary-text')

    def test_import_xyz_preserves_order_duplicates_signed_zero_and_original_bytes(self):
        raw = struct.pack('<fffffffff', 3, -0.0, 4, 3, -0.0, 4, -1, 2, 5)
        result = self.imported(raw, 'xyz-f32-le', source='mockamap')
        self.assertEqual(result['point_cloud'], raw)
        self.assertEqual(result['recipe']['point_count'], 3)
        self.assertEqual(result['recipe']['source'], 'mockamap')

    def test_import_pcd_ascii_reorders_xyz_and_preserves_extra_fields_in_original(self):
        raw = pcd('3 7 1 2 10 20\n6 8 4 5 30 40\n', fields='z intensity x y descriptor',
                  sizes='4 1 4 4 4', types='F U F F F', counts='1 1 1 1 2')
        result = self.imported(raw)
        self.assertEqual(unpack(result['point_cloud']), [(1, 2, 3), (4, 5, 6)])
        self.assertEqual(result['recipe']['input']['fields'], ['z', 'intensity', 'x', 'y', 'descriptor'])

    def test_import_pcd_binary_handles_declared_sizes_and_counts(self):
        raw = pcd(struct.pack('<fIdh', 1.25, 0x11223344, 2.5, 3) + struct.pack('<fIdh', 4.25, 0x55667788, 5.5, 6),
                  encoding='binary', fields='x rgba y z', sizes='4 4 8 2', types='F U F I', counts='1 1 1 1')
        self.assertEqual(unpack(self.imported(raw)['point_cloud']), [(1.25, 2.5, 3), (4.25, 5.5, 6)])

    def test_import_ply_ascii_and_binary_support_extra_properties_and_faces(self):
        properties = 'property uchar red\nproperty double z\nproperty float x\nproperty float y\n'
        suffix = 'element face 1\nproperty list uchar int vertex_indices\n'
        ascii_raw = ply('255 3.5 1.5 2.5\n128 6.5 4.5 5.5\n3 0 1 0\n', properties=properties, suffix=suffix)
        binary_raw = ply(struct.pack('<Bdff', 255, 3.5, 1.5, 2.5) + struct.pack('<Bdff', 128, 6.5, 4.5, 5.5)
                         + struct.pack('<Biii', 3, 0, 1, 0), encoding='binary_little_endian', properties=properties, suffix=suffix)
        for raw in (ascii_raw, binary_raw):
            with self.subTest(binary=raw is binary_raw):
                result = self.imported(raw, 'ply')
                self.assertEqual(unpack(result['point_cloud']), [(1.5, 2.5, 3.5), (4.5, 5.5, 6.5)])

    def test_import_requires_explicit_resolution_and_representation(self):
        path = self.file('source.pcd', pcd('1 2 3\n4 5 6\n'))
        for parameters in [{}, dict(resolution=.1), dict(representation='surface'),
                           dict(resolution=0, representation='surface'),
                           dict(resolution=.1, representation='unknown'),
                           dict(resolution=.1, representation='occupied', frame='map'),
                           dict(resolution=.1, representation='surface', format='compressed-pcd')]:
            with self.subTest(parameters=parameters), self.assertRaises(ValueError):
                sources.generate_scene('bad', 'pointcloud', parameters, input_path=path)
        result = sources.generate_scene('occupied', 'pointcloud', dict(resolution=.2, representation='occupied'), input_path=path)
        self.assertEqual(result['recipe']['representation'], 'occupied')
        self.assertEqual(result['recipe']['resolution'], .2)
        self.assertEqual(result['scene']['obstacles'], [])

    def test_import_rejects_compression_bad_points_and_malformed_payloads(self):
        pcd_cases = [pcd('1 2 3\n4 5 6\n', encoding='binary_compressed'),
                     pcd('nan 2 3\n4 5 6\n'), pcd('1 inf 3\n4 5 6\n'),
                     pcd('1 2 3\n'), pcd('1 2 3 4\n4 5 6\n'),
                     pcd('1 2 3\n4 5 6\n', fields='x x z'),
                     pcd('1 2 3\n4 5 6\n', fields='x y intensity'),
                     pcd(b'\0' * 23, encoding='binary'),
                     pcd(b'\0' * 25, encoding='binary'),
                     pcd('1 2 3\n4 5 6\n', counts='1 1 0'),
                     pcd('1 2 3\n4 5 6\n', count=0), b'not a pcd']
        ply_cases = [ply('1 2 3\n4 5 6\n', encoding='binary_big_endian'),
                     ply('1 2 nan\n4 5 6\n'), ply('1 2 3\n'),
                     ply('1 2 3 4\n4 5 6\n'),
                     ply(b'\0' * 23, encoding='binary_little_endian'),
                     ply(b'\0' * 25, encoding='binary_little_endian'),
                     ply('1 2 3\n4 5 6\n', properties='property float x\nproperty float x\nproperty float z\n'),
                     ply('1 2 3\n4 5 6\n', properties='property long x\nproperty float y\nproperty float z\n')]
        xyz_cases = [b'\0' * 13, struct.pack('<fff', float('nan'), 0, 0), struct.pack('<fff', float('inf'), 0, 0)]
        for format_name, cases in [('pcd', pcd_cases), ('ply', ply_cases), ('xyz-f32-le', xyz_cases)]:
            for index, data in enumerate(cases):
                with self.subTest(format=format_name, case=index), self.assertRaises(ValueError):
                    self.imported(data, format_name)

    def test_producer_budgets_and_irrelevant_input_fields_are_rejected(self):
        path = self.file('map.xyz', struct.pack('<fff', 1, 2, 3))
        parameters = dict(format='xyz-f32-le', resolution=.1, representation='surface')
        with mock.patch.object(sources, 'MAX_ATTACHMENT_BYTES', 8), self.assertRaises(ValueError):
            sources.generate_scene('large', 'pointcloud', parameters, input_path=path)
        with mock.patch.object(sources, 'MAX_ATTACHMENT_BYTES', 20), self.assertRaisesRegex(ValueError, 'Combined'):
            sources.generate_scene('large', 'pointcloud', parameters, input_path=path)
        with mock.patch.object(sources, 'MAX_SCENE_BYTES', 20), self.assertRaisesRegex(ValueError, '1 MiB'):
            sources.generate_scene('large', 'primitive-forest', {})
        with self.assertRaises(ValueError):
            sources.generate_scene('bad', 'primitive-forest', {}, input_path=path)
        with self.assertRaises(ValueError):
            sources.generate_scene('bad', 'pointcloud', parameters, input_path=path, tree_models=[path])
        with self.assertRaises(ValueError):
            sources.generate_scene('bad', 'pointcloud', parameters, input_path=self.root / 'missing')


if __name__ == '__main__':
    unittest.main()
