import copy
import math
from pathlib import Path
import tempfile
import unittest

from xgc2_scene_runtime.document import SceneError, document, geometry
from xgc2_scene_runtime.store import SceneStore, dump_yaml, load


def ring_scene():
    return {'schema': 'xgc2.scene.v1', 'id': 'elliptic', 'frame': 'world', 'obstacles': [{
        'id': 'opening', 'pose': {'position': [3, -2, 4], 'orientation': [.5, .5, .5, .5]},
        'parts': [{'id': 'tube', 'pose': {'position': [1, 2, 3], 'orientation': [.5, -.5, .5, .5]},
                   'geometry': {'type': 'ring', 'radii': [3, 1], 'tubeRadius': 0.8}}]}]}


class ParametricRingTest(unittest.TestCase):
    def test_strict_named_dimensions_and_hole(self):
        # r exceeds the minimum curvature radius 1/3, but remains valid.
        self.assertEqual(geometry({'type': 'ring', 'radii': [3, 1], 'tubeRadius': .8}),
                         {'type': 'ring', 'radii': [3., 1.], 'tubeRadius': .8})
        base = {'type': 'ring', 'radii': [3, 1], 'tubeRadius': .2}
        invalid = [dict(base, radii=v) for v in ([1], [1, 2, 3], [True, 2], [0, 2],
                   [-1, 2], [math.nan, 2], [math.inf, 2], [1e5, 2])]
        invalid += [dict(base, tubeRadius=v) for v in (True, 0, -1, 1, 2, math.inf, math.nan)]
        invalid += [dict(base, **{key: value}) for key, value in
                    [('radius', .2), ('height', 1), ('size', [3, 1, .2]), ('segments', 126), ('params', {})]]
        invalid += [{'type': 'ring', 'radii': [3, 1]}, {'type': 'ring', 'tubeRadius': .2}]
        for value in invalid:
            with self.subTest(value=value), self.assertRaises(SceneError):
                geometry(value)

    def test_save_reopen_edit_undo_preserves_one_part_and_poses(self):
        expected = document(ring_scene())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'ring.yaml'
            path.write_text(dump_yaml(expected))
            store = SceneStore(load(path)[0], source=path)
            self.assertEqual(store.document, expected)
            changed = copy.deepcopy(expected['obstacles'][0])
            changed['parts'][0]['geometry']['radii'] = [4, 1.25]
            def command(operation, **values):
                return store.command(dict(requestId=str(store.revision)+operation,
                    expectedEpoch=store.epoch, expectedRevision=store.revision, operation=operation, **values))
            result = command('update', obstacle=changed)
            self.assertTrue(result['success'], result)
            self.assertEqual(load(path)[0]['obstacles'][0]['parts'], changed['parts'])
            self.assertTrue(command('undo')['success'])
            self.assertEqual(SceneStore(load(path)[0], source=path).document, expected)
            self.assertEqual(len(expected['obstacles'][0]['parts']), 1)


if __name__ == '__main__':
    unittest.main()
