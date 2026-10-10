"""Place sealed scene elements using the runtime's canonical geometry operations.

Finite CLI: python3 -m xgc2_scene_runtime.assembly (JSON stdin/stdout).
The archive owner supplies verified document bytes; this module opens no files.
"""

import json
import math
import sys

import yaml

from .document import SceneError, document, identifier, number
from .motion import compose, rotate
from .store import MAX_DOCUMENT_BYTES, SceneLoader, dump_yaml, unique_object


def assemble(request, documents, extrinsic):
    if request.get('fidelity') not in ('geometry-only', 'gazebo-world'):
        raise SceneError('fidelity must be geometry-only or gazebo-world')
    task = identifier(request.get('task', '').strip(), 'Task')
    if len(task) > 64:
        raise SceneError('Task must have at most 64 characters')
    elements = request.get('elements')
    if not isinstance(elements, list) or not 2 <= len(elements) <= 64 or len(documents) != len(elements):
        raise SceneError('An assembled world needs 2 to 64 elements')
    obstacles, ids, frame = [], set(), None
    for element, source in zip(elements, documents):
        eid = identifier(element['id'], 'Element ID')
        if '.' in eid or eid in ids:
            raise SceneError('Element IDs must be unique and contain no dots')
        ids.add(eid)
        x, y, yaw = [number(element.get(key, 0), key, -1e4, 1e4) for key in ('x', 'y', 'yaw')]
        placement = {'position': [x, y, 0], 'orientation': [0, 0, math.sin(yaw/2), math.cos(yaw/2)]}
        scene = document(yaml.load(source, Loader=SceneLoader))
        if frame is not None and frame != scene['frame']:
            raise SceneError('All elements must use the same world frame')
        frame = scene['frame']
        for original in scene['obstacles']:
            placed = dict(original, id=identifier(eid + '.' + original['id']),
                          name=eid + ' ' + original['name'], pose=compose(placement, original['pose']))
            movement = dict(original['motion'])
            for key in ('point_a', 'point_b', 'center'):
                if key in movement:
                    delta = rotate(placement['orientation'], movement[key])
                    movement[key] = [a+b for a, b in zip(placement['position'], delta)]
            for key in ('linear', 'angular'):
                if key in movement:
                    movement[key] = rotate(placement['orientation'], movement[key])
            if movement['type'] == 'circle':
                movement['phase'] += yaw
            placed['motion'] = movement
            obstacles.append(placed)
    camera = yaml.load(extrinsic, Loader=SceneLoader)
    if not isinstance(camera, dict) or camera.get('parent_frame') != frame:
        raise SceneError('cameraFrom extrinsic parent frame must be the world frame ' + frame)
    name = request['name'].strip().lower().replace(' ', '-').replace('_', '-')
    # The canonical validator owns all geometry and aggregate limits, including
    # the transformed motion's initial pose. No separate assembly schema exists.
    result = document({'schema': 'xgc2.scene.v1', 'id': name, 'frame': frame, 'obstacles': obstacles})
    encoded = dump_yaml(result)
    if len(encoded.encode('utf-8')) > MAX_DOCUMENT_BYTES:
        raise SceneError('Assembled scene exceeds the supported size')
    return {'sceneDocument': encoded, 'task': task}


def main():
    try:
        raw = sys.stdin.buffer.read(4*MAX_DOCUMENT_BYTES+1)
        if len(raw) > 4*MAX_DOCUMENT_BYTES:
            raise SceneError('Assembly input exceeds the supported size')
        value = json.loads(raw, object_pairs_hook=unique_object,
                           parse_constant=lambda _: (_ for _ in ()).throw(SceneError('Nonfinite JSON')))
        result = assemble(value['request'], value['documents'], value['extrinsic'])
        sys.stdout.write(json.dumps(result, allow_nan=False, separators=(',', ':')))
        return 0
    except (SceneError, yaml.YAMLError, KeyError, TypeError, AttributeError, ValueError) as error:
        sys.stderr.write(str(error) + '\n')
        return 2


if __name__ == '__main__':
    sys.exit(main())
