"""Canonical scene geometry. Validation does not depend on a simulator or ROS."""

import math
import re

SCHEMA = 'xgc2.scene.v1'
IDENTITY = {'position': [0.0, 0.0, 0.0], 'orientation': [0.0, 0.0, 0.0, 1.0]}
ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$')


class SceneError(ValueError):
    pass


def fields(value, allowed, required=()):
    if not isinstance(value, dict):
        raise SceneError('Expected an object')
    unknown = set(value) - set(allowed)
    missing = set(required) - set(value)
    if unknown or missing:
        raise SceneError('Invalid fields: unknown={}, missing={}'.format(sorted(map(str, unknown)), sorted(missing)))
    return value


def number(value, label, minimum=-1e6, maximum=1e6):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SceneError('{} must be finite'.format(label))
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise SceneError('{} must be finite'.format(label))
    if not minimum <= value <= maximum:
        raise SceneError('{} is outside the supported range'.format(label))
    return float(value)


def vector(value, label, count=3):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise SceneError('{} must have {} components'.format(label, count))
    return [number(x, label) for x in value]


def identifier(value, label='ID'):
    if not isinstance(value, str) or not ID_PATTERN.fullmatch(value):
        raise SceneError('{} must be a stable nonempty identifier'.format(label))
    return value


def pose(value):
    fields(value, ('position', 'orientation'))
    position = vector(value.get('position', [0, 0, 0]), 'position')
    rotation = vector(value.get('orientation', [0, 0, 0, 1]), 'orientation', 4)
    norm = math.sqrt(sum(x * x for x in rotation))
    if norm < 1e-12:
        raise SceneError('Orientation quaternion must be nonzero')
    return {'position': position, 'orientation': [x / norm for x in rotation]}


def subtract(a, b):
    return [x - y for x, y in zip(a, b)]


def cross(a, b):
    return [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]]


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def convex(value):
    vertices = value.get('vertices')
    indices = value.get('triangles')
    if not isinstance(vertices, list) or not 4 <= len(vertices) <= 4096:
        raise SceneError('Convex geometry requires 4 to 4096 vertices')
    vertices = [vector(p, 'vertex') for p in vertices]
    if len(set(tuple(p) for p in vertices)) != len(vertices):
        raise SceneError('Convex geometry has duplicate vertices')
    if not isinstance(indices, list) or len(indices) < 12 or len(indices) > 24576 or len(indices) % 3:
        raise SceneError('Convex geometry requires triangle index triplets')
    if any(isinstance(i, bool) or not isinstance(i, int) or not 0 <= i < len(vertices) for i in indices):
        raise SceneError('Convex triangle index is invalid')
    if set(indices) != set(range(len(vertices))):
        raise SceneError('Every convex vertex must belong to its surface')
    center = [sum(p[i] for p in vertices)/len(vertices) for i in range(3)]
    edges = {}
    triangles = []
    volume = 0.0
    for offset in range(0, len(indices), 3):
        a, b, c = indices[offset:offset+3]
        normal = cross(subtract(vertices[b], vertices[a]), subtract(vertices[c], vertices[a]))
        magnitude = math.sqrt(dot(normal, normal))
        if magnitude < 1e-12:
            raise SceneError('Convex geometry has a degenerate face')
        if dot(normal, subtract(center, vertices[a])) > 0:
            b, c = c, b
            normal = [-x for x in normal]
        if any(dot(normal, subtract(p, vertices[a])) > 1e-8*magnitude for p in vertices):
            raise SceneError('Geometry is not convex; represent it as separate convex parts')
        volume += abs(dot(normal, subtract(center, vertices[a])))/6.0
        triangles.extend((a, b, c))
        for edge in ((a, b), (b, c), (c, a)):
            edges[edge] = edges.get(edge, 0) + 1
    if volume < 1e-12 or any(count != 1 or edges.get((b, a)) != 1 for (a, b), count in edges.items()):
        raise SceneError('Convex surface must be a closed nonzero-volume manifold')
    return {'type': 'convex', 'vertices': vertices, 'triangles': triangles}


def geometry(value):
    if not isinstance(value, dict):
        raise SceneError('Geometry must be an object')
    kind = value.get('type')
    if kind == 'box':
        fields(value, ('type', 'size'), ('size',))
        size = vector(value['size'], 'box size')
        if any(x <= 0 for x in size):
            raise SceneError('Box side lengths must be positive')
        return {'type': kind, 'size': size}
    if kind in ('sphere', 'cylinder', 'capsule'):
        allowed = ('type', 'radius') if kind == 'sphere' else ('type', 'radius', 'height')
        fields(value, allowed, allowed)
        result = {'type': kind, 'radius': number(value['radius'], 'radius', 1e-6, 1e4)}
        if kind != 'sphere':
            result['height'] = number(value['height'], 'height', 0 if kind == 'capsule' else 1e-6, 1e4)
        return result
    if kind == 'convex':
        fields(value, ('type', 'vertices', 'triangles'), ('vertices', 'triangles'))
        return convex(value)
    raise SceneError('Unsupported geometry {!r}; no shape substitution is performed'.format(kind))


def motion(value):
    if not isinstance(value, dict):
        raise SceneError('Motion must be an object')
    kind = value.get('type', 'hold')
    if kind == 'hold':
        fields(value, ('type',))
        return {'type': kind}
    if kind == 'constant_twist':
        fields(value, ('type', 'linear', 'angular'))
        return {'type': kind, 'linear': vector(value.get('linear', [0, 0, 0]), 'linear velocity'),
                'angular': vector(value.get('angular', [0, 0, 0]), 'angular velocity')}
    if kind == 'ping_pong':
        fields(value, ('type', 'point_a', 'point_b', 'speed'), ('point_a', 'point_b', 'speed'))
        a, b = vector(value['point_a'], 'point A'), vector(value['point_b'], 'point B')
        if sum((x-y)**2 for x, y in zip(a, b)) < 1e-12:
            raise SceneError('Ping-pong endpoints must differ')
        return {'type': kind, 'point_a': a, 'point_b': b, 'speed': number(value['speed'], 'speed', 1e-6, 1000)}
    if kind == 'circle':
        fields(value, ('type', 'center', 'radius', 'angular_speed', 'phase'), ('center', 'radius', 'angular_speed'))
        return {'type': kind, 'center': vector(value['center'], 'circle center'),
                'radius': number(value['radius'], 'circle radius', 1e-6, 1e4),
                'angular_speed': number(value['angular_speed'], 'angular speed', -1000, 1000),
                'phase': number(value.get('phase', 0), 'phase')}
    raise SceneError('Unsupported motion {!r}'.format(kind))


def obstacle(value):
    fields(value, ('id', 'name', 'pose', 'parts', 'motion'), ('id', 'parts'))
    oid = identifier(value['id'], 'Obstacle ID')
    name = value.get('name', oid)
    if not isinstance(name, str) or not name.strip() or len(name) > 256:
        raise SceneError('Obstacle name must have 1 to 256 characters')
    parts = value['parts']
    if not isinstance(parts, list) or not 1 <= len(parts) <= 128:
        raise SceneError('Obstacle requires 1 to 128 parts')
    normalized = []
    ids = set()
    for part in parts:
        fields(part, ('id', 'pose', 'geometry', 'color'), ('id', 'geometry'))
        pid = identifier(part['id'], 'Part ID')
        if pid in ids:
            raise SceneError('Duplicate part ID {}'.format(pid))
        ids.add(pid)
        color = vector(part.get('color', [0.9, 0.6, 0.1, 0.65]), 'color', 4)
        if any(x < 0 or x > 1 for x in color):
            raise SceneError('Color channels must be between 0 and 1')
        normalized.append({'id': pid, 'pose': pose(part.get('pose', {})),
                           'geometry': geometry(part['geometry']), 'color': color})
    initial = pose(value.get('pose', {}))
    movement = motion(value.get('motion', {'type': 'hold'}))
    start = initial['position']
    if movement['type'] == 'ping_pong':
        start = movement['point_a']
    elif movement['type'] == 'circle':
        center, radius, phase = movement['center'], movement['radius'], movement['phase']
        start = [center[0]+radius*math.cos(phase), center[1]+radius*math.sin(phase), center[2]]
    if any(abs(a-b) > 1e-6 for a, b in zip(start, initial['position'])):
        raise SceneError('Initial obstacle position must match its motion path start')
    return {'id': oid, 'name': name, 'pose': initial, 'parts': normalized, 'motion': movement}


def document(value):
    fields(value, ('schema', 'id', 'frame', 'obstacles'), ('schema', 'id', 'frame', 'obstacles'))
    if value['schema'] != SCHEMA:
        raise SceneError('Unsupported scene schema')
    identifier(value['id'], 'Scene ID')
    frame = value['frame']
    if not isinstance(frame, str) or len(frame) > 160 or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', frame):
        raise SceneError('Scene frame must be explicit and have no leading slash')
    if not isinstance(value['obstacles'], list) or len(value['obstacles']) > 512:
        raise SceneError('Scene supports at most 512 obstacles')
    obstacles = [obstacle(item) for item in value['obstacles']]
    if len({item['id'] for item in obstacles}) != len(obstacles):
        raise SceneError('Duplicate obstacle ID')
    if sum(len(item['parts']) for item in obstacles) > 4096:
        raise SceneError('Scene supports at most 4096 convex parts')
    return {'schema': SCHEMA, 'id': value['id'], 'frame': frame, 'obstacles': obstacles}
