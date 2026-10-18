"""Bounded, offline producers for the existing xgc2.scene.v1 asset contract.

Only the Python standard library is used. No input file is modified and no
subprocess, network, ROS or simulator is started. ``resources`` contains bytes
under asset-relative paths; callers own sealing those bytes in the existing
scene asset API. The catalog is also the parameter validation authority.

Primitive forests are platform generators, not byte reproductions of an
upstream random_forest_sensing version. The default tree is a small platform
branching template. External, already exported OBJ assets can be supplied by
an offline tool such as Infinigen; no Infinigen installation is assumed.
"""

import bisect
import hashlib
import io
import json
import math
from pathlib import Path
import random
import re
import struct


MAX_ATTACHMENT_BYTES = 48 * 1024 * 1024
MAX_SCENE_BYTES = 1024 * 1024
MAX_POINTS = MAX_ATTACHMENT_BYTES // 12
MAX_OBSTACLES = 512
MAX_MODELS = 512
MAX_OBJ_VERTICES = 200000
MAX_OBJ_TRIANGLES = 400000
_HEADER_BYTES = 65536
_COORDINATE_LIMIT = 1000000.0


def _parameter(name, title, default, kind='number', minimum=None, maximum=None, **extra):
    result = dict(name=name, title=title, default=default, type=kind)
    if minimum is not None:
        result['minimum'] = minimum
    if maximum is not None:
        result['maximum'] = maximum
    result.update(extra)
    return result


_LAYOUT = [
    _parameter('seed', 'Seed', 0, 'integer', 0, 2**53 - 1),
    _parameter('count', 'Obstacle count', 40, 'integer', 0, MAX_OBSTACLES),
    _parameter('x_min', 'X minimum (m)', -20.0, minimum=-10000, maximum=10000),
    _parameter('x_max', 'X maximum (m)', 20.0, minimum=-10000, maximum=10000),
    _parameter('y_min', 'Y minimum (m)', -20.0, minimum=-10000, maximum=10000),
    _parameter('y_max', 'Y maximum (m)', 20.0, minimum=-10000, maximum=10000),
    _parameter('min_distance', 'Minimum center spacing (m)', 1.5, minimum=0, maximum=10000),
]
_FRAME = _parameter('frame', 'Coordinate frame', 'world', 'string', enum=['world'])
_DENSITY = _parameter('samples_per_m2', 'Reference surface samples per square meter', 20.0,
                      minimum=0.001, maximum=10000)

SOURCE_CATALOG = [
    dict(id='primitive-forest', title='Columns and ring gates', input=None, parameters=_LAYOUT + [
        _parameter('ring_count', 'Ring gate count', 8, 'integer', 0, MAX_OBSTACLES),
        _parameter('radius_min', 'Minimum column radius (m)', 0.2, minimum=0.001, maximum=100),
        _parameter('radius_max', 'Maximum column radius (m)', 0.5, minimum=0.001, maximum=100),
        _parameter('height_min', 'Minimum column height (m)', 2.0, minimum=0.001, maximum=100),
        _parameter('height_max', 'Maximum column height (m)', 4.0, minimum=0.001, maximum=100),
        _parameter('ring_radius_x', 'Ring horizontal radius (m)', 1.2, minimum=0.001, maximum=100),
        _parameter('ring_radius_y', 'Ring vertical radius (m)', 0.9, minimum=0.001, maximum=100),
        _parameter('tube_radius', 'Ring tube radius (m)', 0.12, minimum=0.001, maximum=100),
        _DENSITY,
        _FRAME,
    ]),
    dict(id='tree-forest', title='Branching tree forest', input='models', parameters=[
        dict(item, default=30 if item['name'] == 'count' else
             2.0 if item['name'] == 'min_distance' else item['default']) for item in _LAYOUT
    ] + [
        _parameter('scale_min', 'Minimum tree scale', 0.8, minimum=0.01, maximum=100),
        _parameter('scale_max', 'Maximum tree scale', 1.2, minimum=0.01, maximum=100),
        _parameter('scale_variants', 'Reusable scale variants', 3, 'integer', 1, 8),
        _DENSITY,
        _FRAME,
    ]),
]
_IMPORT_PARAMETERS = [
    _parameter('format', 'Input format', 'pcd', 'string', enum=['pcd', 'ply', 'xyz-f32-le']),
    _parameter('resolution', 'Declared input resolution (m)', 0.1,
               minimum=0.000001, maximum=100, required=True),
    _parameter('representation', 'Point meaning', 'surface', 'string',
               enum=['surface', 'occupied'], required=True),
    _FRAME,
]
SOURCE_CATALOG.extend([
    dict(id='pointcloud', title='Import point cloud', input='pointcloud', parameters=_IMPORT_PARAMETERS),
    dict(id='mockamap', title='Import generated mockamap output', input='pointcloud', parameters=_IMPORT_PARAMETERS),
])
DEFAULT_PARAMETERS = {item['id']: {p['name']: p['default'] for p in item['parameters']}
                      for item in SOURCE_CATALOG}


def _validated(source, parameters):
    descriptor = next((item for item in SOURCE_CATALOG if item['id'] == source), None)
    if descriptor is None:
        raise ValueError('Unsupported scene source')
    if not isinstance(parameters, dict) or any(not isinstance(key, str) for key in parameters):
        raise ValueError('parameters must be an object with string keys')
    specs = {item['name']: item for item in descriptor['parameters']}
    if set(parameters) - set(specs):
        raise ValueError('Unknown scene parameters: ' + ', '.join(sorted(set(parameters) - set(specs))))
    result = {}
    for name, spec in specs.items():
        if spec.get('required') and name not in parameters:
            raise ValueError(name + ' must be explicitly declared')
        value = parameters.get(name, spec['default'])
        if spec['type'] == 'string':
            if not isinstance(value, str) or value not in spec['enum']:
                raise ValueError(name + ' must be one of ' + ', '.join(spec['enum']))
        else:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(name + ' must be a finite number')
            try:
                finite = math.isfinite(value)
            except OverflowError:
                finite = False
            if not finite:
                raise ValueError(name + ' must be a finite number')
            if spec['type'] == 'integer' and type(value) is not int:
                raise ValueError(name + ' must be an integer')
            if not spec['minimum'] <= value <= spec['maximum']:
                raise ValueError(name + ' is outside the supported range')
        result[name] = value
    return result


def _read(path):
    if not isinstance(path, (str, Path)) or not str(path) or '\x00' in str(path):
        raise ValueError('An input file path is required')
    try:
        with Path(path).open('rb') as handle:
            data = handle.read(MAX_ATTACHMENT_BYTES + 1)
    except OSError as error:
        raise ValueError('Input file could not be read') from error
    if not data or len(data) > MAX_ATTACHMENT_BYTES:
        raise ValueError('Input files must be nonempty and at most 48 MiB')
    return data


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _add_resource(resources, file, data):
    if len(data) > MAX_ATTACHMENT_BYTES:
        raise ValueError('Scene attachment exceeds the existing 48 MiB limit')
    if sum(len(value) for name, value in resources.items() if name != file) + len(data) > MAX_ATTACHMENT_BYTES:
        raise ValueError('Combined scene resources exceed the bounded 48 MiB producer budget')
    resources[file] = data


def _scene(name):
    if not isinstance(name, str) or name != name.strip() or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9 _-]{0,79}', name):
        raise ValueError('Scene name must be 1 to 80 readable ASCII letters, digits, spaces, underscores or hyphens')
    return dict(schema='xgc2.scene.v1', id=name.replace(' ', '-'), frame='world',
                ground={'mode': 'none'}, obstacles=[])


def _pair(parameters, low, high):
    if parameters[low] > parameters[high]:
        raise ValueError(low + ' must not exceed ' + high)


def _layout(parameters, count):
    for low, high in [('x_min', 'x_max'), ('y_min', 'y_max')]:
        if parameters[low] >= parameters[high]:
            raise ValueError(low + ' must be smaller than ' + high)
    if count > MAX_OBSTACLES:
        raise ValueError('The common scene contract supports at most 512 obstacles')
    rng = random.Random(parameters['seed'])
    spacing = parameters['min_distance']
    bins, result = {}, []
    attempts = max(1000, count * 100)
    for _ in range(attempts):
        if len(result) == count:
            return result, rng
        x = rng.uniform(parameters['x_min'], parameters['x_max'])
        y = rng.uniform(parameters['y_min'], parameters['y_max'])
        if spacing:
            cell = (math.floor(x / spacing), math.floor(y / spacing))
            nearby = (point for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                      for point in bins.get((cell[0] + dx, cell[1] + dy), ()))
            if any((x - a)**2 + (y - b)**2 < spacing**2 for a, b in nearby):
                continue
            bins.setdefault(cell, []).append((x, y))
        result.append((x, y))
    raise ValueError('Requested center spacing could not be satisfied within the bounded layout budget')


def _primitive(scene, parameters, recipe):
    _pair(parameters, 'radius_min', 'radius_max')
    _pair(parameters, 'height_min', 'height_max')
    if parameters['tube_radius'] >= min(parameters['ring_radius_x'], parameters['ring_radius_y']):
        raise ValueError('Ring tube radius must be smaller than both ring radii to retain its hole')
    positions, rng = _layout(parameters, parameters['count'] + parameters['ring_count'])
    surfaces, sample_count = [], 0
    for index, (x, y) in enumerate(positions):
        if index < parameters['count']:
            height = rng.uniform(parameters['height_min'], parameters['height_max'])
            geometry = dict(type='cylinder', radius=rng.uniform(parameters['radius_min'], parameters['radius_max']), height=height)
            pose = dict(position=[x, y, height / 2])
            oid = 'column-{}'.format(index)
        else:
            geometry = dict(type='ring', radii=[parameters['ring_radius_x'], parameters['ring_radius_y']],
                            tubeRadius=parameters['tube_radius'])
            yaw = rng.uniform(-math.pi, math.pi)
            # Local XY ring plane is rotated upright about X, then about Z.
            half = math.sqrt(0.5)
            pose = dict(position=[x, y, parameters['ring_radius_y'] + parameters['tube_radius']],
                        orientation=[half * math.cos(yaw / 2), half * math.sin(yaw / 2),
                                     half * math.sin(yaw / 2), half * math.cos(yaw / 2)])
            oid = 'gate-{}'.format(index - parameters['count'])
        scene['obstacles'].append(dict(id=oid, pose=pose, parts=[dict(id='body', geometry=geometry)]))
        vertices, triangles = _primitive_surface(geometry)
        cumulative, total = _distribution(vertices, triangles, 1)
        count = math.ceil(total * parameters['samples_per_m2'])
        sample_count += count
        if sample_count > MAX_POINTS:
            raise ValueError('Surface sampling exceeds the 48 MiB point cloud budget; reduce count or samples_per_m2')
        surfaces.append((vertices, triangles, 1, pose, cumulative, total, count))
    recipe.update(geometry_status='parameterized', point_cloud_role='reference_surface',
                  sampling='triangle-area weighted samples of a fixed cylinder/ring surface tessellation',
                  tessellation=dict(cylinder_sides=32, ring_segments=64, tube_segments=16),
                  point_count=sample_count,
                  reproduction='platform parameterized generator; not upstream random_forest_sensing byte reproduction')
    return _sample_surfaces(surfaces, parameters['seed']) if sample_count else None


def _coordinates(values):
    result = tuple(float(value) for value in values)
    if len(result) != 3 or any(not math.isfinite(value) or abs(value) > _COORDINATE_LIMIT for value in result):
        raise ValueError('Every XYZ point must be finite and inside the supported coordinate range')
    return result


def _area(a, b, c):
    u, v = [b[i] - a[i] for i in range(3)], [c[i] - a[i] for i in range(3)]
    return math.sqrt(sum(value * value for value in
                         (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]))) / 2


def _primitive_surface(geometry):
    vertices, triangles = [], []
    if geometry['type'] == 'cylinder':
        sides, radius, height = 32, geometry['radius'], geometry['height']
        for z in (-height / 2, height / 2):
            vertices.extend((radius * math.cos(2 * math.pi * index / sides),
                             radius * math.sin(2 * math.pi * index / sides), z) for index in range(sides))
        vertices.extend(((0, 0, -height / 2), (0, 0, height / 2)))
        for index in range(sides):
            following = (index + 1) % sides
            triangles.extend(((index, following, sides + following), (index, sides + following, sides + index),
                              (2 * sides, following, index), (2 * sides + 1, sides + index, sides + following)))
    else:
        a, b = geometry['radii']
        tube, sides, cross_sections = geometry['tubeRadius'], 64, 16
        for index in range(sides):
            angle = 2 * math.pi * index / sides
            x, y = a * math.cos(angle), b * math.sin(angle)
            nx, ny = b * math.cos(angle), a * math.sin(angle)
            normal_length = math.hypot(nx, ny)
            nx, ny = nx / normal_length, ny / normal_length
            for section in range(cross_sections):
                phase = 2 * math.pi * section / cross_sections
                vertices.append((x + tube * math.cos(phase) * nx, y + tube * math.cos(phase) * ny,
                                 tube * math.sin(phase)))
        for index in range(sides):
            for section in range(cross_sections):
                ia, ib = index * cross_sections + section, ((index + 1) % sides) * cross_sections + section
                ic = ((index + 1) % sides) * cross_sections + (section + 1) % cross_sections
                id_ = index * cross_sections + (section + 1) % cross_sections
                triangles.extend(((ia, ib, ic), (ia, ic, id_)))
    return vertices, triangles


def _distribution(vertices, triangles, scale):
    cumulative, total = [], 0.0
    for a, b, c in triangles:
        area = _area(vertices[a], vertices[b], vertices[c]) * scale**2
        if not math.isfinite(area) or area <= 0:
            raise ValueError('Surface geometry has a nonfinite or degenerate triangle')
        total += area
        cumulative.append(total)
    return cumulative, total


def _sample_surfaces(instances, seed):
    sampler, cloud = random.Random(seed ^ 0x53414d504c45), bytearray()
    for vertices, triangles, scale, pose, cumulative, total, count in instances:
        position = pose['position']
        qx, qy, qz, qw = pose.get('orientation', (0, 0, 0, 1))
        rotation = ((1 - 2 * (qy*qy + qz*qz), 2 * (qx*qy - qz*qw), 2 * (qx*qz + qy*qw)),
                    (2 * (qx*qy + qz*qw), 1 - 2 * (qx*qx + qz*qz), 2 * (qy*qz - qx*qw)),
                    (2 * (qx*qz - qy*qw), 2 * (qy*qz + qx*qw), 1 - 2 * (qx*qx + qy*qy)))
        for _ in range(count):
            triangle = triangles[min(bisect.bisect_right(cumulative, sampler.random() * total), len(triangles) - 1)]
            a, b, c = [vertices[item] for item in triangle]
            root, v = math.sqrt(sampler.random()), sampler.random()
            weights = (1 - root, root * (1 - v), root * v)
            point = [scale * sum(weights[j] * vertex[i] for j, vertex in enumerate((a, b, c))) for i in range(3)]
            world = _coordinates(position[i] + sum(rotation[i][j] * point[j] for j in range(3)) for i in range(3))
            cloud.extend(struct.pack('<fff', *world))
    return bytes(cloud)


def _obj(data):
    vertices, triangles = [], []
    for raw_line in io.BytesIO(data):
        if len(raw_line) > MAX_SCENE_BYTES:
            raise ValueError('OBJ line exceeds the bounded parser budget')
        try:
            line = raw_line.decode('utf-8')
        except UnicodeError as error:
            raise ValueError('OBJ input must be UTF-8 text') from error
        fields = line.split('#', 1)[0].split()
        if not fields:
            continue
        tag, values = fields[0], fields[1:]
        if tag == 'v':
            if len(values) not in (3, 4):
                raise ValueError('OBJ vertices require XYZ and optional homogeneous W')
            vertex = _coordinates(values[:3])
            if len(values) == 4:
                w = float(values[3])
                if not math.isfinite(w) or w == 0:
                    raise ValueError('OBJ homogeneous W must be finite and nonzero')
                vertex = _coordinates(value / w for value in vertex)
            vertices.append(vertex)
            if len(vertices) > MAX_OBJ_VERTICES:
                raise ValueError('OBJ vertex count exceeds the bounded input budget')
        elif tag == 'f':
            if not 3 <= len(values) <= 128:
                raise ValueError('OBJ faces require 3 to 128 vertices')
            face = []
            for value in values:
                try:
                    index = int(value.split('/', 1)[0])
                except ValueError as error:
                    raise ValueError('Invalid OBJ position index') from error
                index = index - 1 if index > 0 else len(vertices) + index if index < 0 else -1
                if not 0 <= index < len(vertices):
                    raise ValueError('OBJ position index is outside the declared vertices')
                face.append(index)
            if len(set(face)) != len(face):
                raise ValueError('OBJ faces cannot repeat a vertex')
            # Fan triangulation is exact only for convex planar polygons. Reject
            # other polygons explicitly rather than change their geometry.
            a, b, c = [vertices[index] for index in face[:3]]
            u, v = [b[i] - a[i] for i in range(3)], [c[i] - a[i] for i in range(3)]
            normal = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            length = math.sqrt(sum(value * value for value in normal))
            if length <= 1e-12:
                raise ValueError('OBJ contains a degenerate face')
            normal = tuple(value / length for value in normal)
            tolerance = 1e-7 * max(1, *(abs(value) for index in face for value in vertices[index]))
            if any(abs(sum((vertices[index][i] - a[i]) * normal[i] for i in range(3))) > tolerance for index in face):
                raise ValueError('OBJ polygon is nonplanar; triangulate it before importing')
            for index in range(len(face)):
                pa, pb, pc = [vertices[face[(index + offset) % len(face)]] for offset in range(3)]
                u, v = [pb[i] - pa[i] for i in range(3)], [pc[i] - pb[i] for i in range(3)]
                cross = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
                if sum(cross[i] * normal[i] for i in range(3)) <= 1e-12:
                    raise ValueError('OBJ polygon is degenerate or concave; triangulate it before importing')
            triangles.extend((face[0], face[index], face[index + 1]) for index in range(1, len(face) - 1))
            if len(triangles) > MAX_OBJ_TRIANGLES:
                raise ValueError('OBJ triangle count exceeds the bounded input budget')
        elif tag not in ('vt', 'vn', 'vp', 'o', 'g', 's', 'usemtl', 'mtllib'):
            raise ValueError('Unsupported OBJ statement: ' + tag)
    if not vertices or not triangles:
        raise ValueError('OBJ requires vertices and nondegenerate triangle faces')
    return vertices, triangles


def _obj_bytes(vertices, triangles, scale):
    lines = ['# Platform geometry-only OBJ; uniform scale baked into vertices.']
    for vertex in vertices:
        scaled = _coordinates(value * scale for value in vertex)
        lines.append('v ' + ' '.join(format(value, '.17g') for value in scaled))
    lines.extend('f {} {} {}'.format(*(value + 1 for value in face)) for face in triangles)
    return ('\n'.join(lines) + '\n').encode('ascii')


def _default_tree():
    vertices, triangles = [], []
    # Closed tapered branch prisms. This intentionally small template contains
    # a trunk and six branches; it does not claim botanical or visual fidelity.
    def branch(start, end, radius, tip):
        axis = tuple(end[i] - start[i] for i in range(3))
        length = math.sqrt(sum(value * value for value in axis))
        axis = tuple(value / length for value in axis)
        reference = (0, 1, 0) if abs(axis[1]) < 0.9 else (1, 0, 0)
        u = (axis[1] * reference[2] - axis[2] * reference[1], axis[2] * reference[0] - axis[0] * reference[2], axis[0] * reference[1] - axis[1] * reference[0])
        norm = math.sqrt(sum(value * value for value in u))
        u = tuple(value / norm for value in u)
        v = (axis[1] * u[2] - axis[2] * u[1], axis[2] * u[0] - axis[0] * u[2], axis[0] * u[1] - axis[1] * u[0])
        base = len(vertices)
        sides = 6
        for center, r in ((start, radius), (end, tip)):
            for index in range(sides):
                angle = 2 * math.pi * index / sides
                vertices.append(tuple(center[i] + r * (u[i] * math.cos(angle) + v[i] * math.sin(angle)) for i in range(3)))
        for index in range(sides):
            following = (index + 1) % sides
            triangles.extend(((base + index, base + following, base + sides + following),
                              (base + index, base + sides + following, base + sides + index)))
        for index in range(1, sides - 1):
            triangles.extend(((base, base + index + 1, base + index),
                              (base + sides, base + sides + index, base + sides + index + 1)))
    branch((0, 0, 0), (0, 0, 4), 0.2, 0.08)
    for index in range(6):
        angle = index * 2 * math.pi / 6
        branch((0, 0, 1.4 + index * 0.3),
               (1.3 * math.cos(angle), 1.3 * math.sin(angle), 2.3 + index * 0.3), 0.07, 0.025)
    return vertices, triangles


def _trees(scene, parameters, tree_models, resources, recipe):
    _pair(parameters, 'scale_min', 'scale_max')
    positions, rng = _layout(parameters, parameters['count'])
    if not isinstance(tree_models, (list, tuple)) or len(tree_models) > 64:
        raise ValueError('tree_models must be a sequence of at most 64 OBJ paths')
    inputs, seen = [], set()
    if tree_models:
        for path in tree_models:
            raw = _read(path)
            digest = _sha(raw)
            if digest in seen:
                continue
            seen.add(digest)
            vertices, triangles = _obj(raw)
            original = 'originals/tree-{}.obj'.format(len(inputs))
            _add_resource(resources, original, raw)
            inputs.append((vertices, triangles, dict(file=original, sha256=digest)))
        recipe['tree_template'] = 'external-obj'
    else:
        vertices, triangles = _default_tree()
        inputs.append((vertices, triangles, dict(producer='platform-branching-template-v1')))
        recipe['tree_template'] = 'platform-branching-template-v1'
    variants = parameters['scale_variants']
    scales = [parameters['scale_min']] if parameters['scale_min'] == parameters['scale_max'] else [
        parameters['scale_min'] + (parameters['scale_max'] - parameters['scale_min']) * index / (variants - 1)
        for index in range(variants)] if variants > 1 else [parameters['scale_min']]
    if len(inputs) * len(scales) > MAX_MODELS:
        raise ValueError('Tree scale variants exceed the common model budget')
    models, distributions, instances = {}, {}, []
    sample_count = 0
    for index, (x, y) in enumerate(positions):
        selected, variant = rng.randrange(len(inputs)), rng.randrange(len(scales))
        vertices, triangles, _ = inputs[selected]
        scale = scales[variant]
        model = 'tree-{}-scale-{}'.format(selected, variant)
        if model not in models:
            file = 'models/{}.obj'.format(model)
            _add_resource(resources, file, _obj_bytes(vertices, triangles, scale))
            models[model] = {'resource': dict(format='mesh', file=file)}
            distributions[model] = _distribution(vertices, triangles, scale)
        cumulative, total = distributions[model]
        count = math.ceil(total * parameters['samples_per_m2'])
        sample_count += count
        if sample_count > MAX_POINTS:
            raise ValueError('Surface sampling exceeds the 48 MiB point cloud budget; reduce count or samples_per_m2')
        yaw = rng.uniform(-math.pi, math.pi)
        pose = dict(position=[x, y, 0.0], orientation=[0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
        scene['obstacles'].append(dict(id='tree-{}'.format(index), model=model, pose=pose))
        instances.append((vertices, triangles, scale, pose, cumulative, total, count))
    if sample_count * 12 + sum(len(data) for data in resources.values()) > MAX_ATTACHMENT_BYTES:
        raise ValueError('Combined scene resources exceed the bounded 48 MiB producer budget')
    scene['models'] = models
    recipe.update(tree_inputs=[item[2] for item in inputs], scales=scales,
                  sampling='independent triangle-area weighted, uniform barycentric surface samples',
                  point_count=sample_count, point_cloud_role='reference_surface', geometry_status='model_resources')
    return _sample_surfaces(instances, parameters['seed']) if sample_count else None


_SCALARS = {
    'char': ('b', 1), 'int8': ('b', 1), 'uchar': ('B', 1), 'uint8': ('B', 1),
    'short': ('h', 2), 'int16': ('h', 2), 'ushort': ('H', 2), 'uint16': ('H', 2),
    'int': ('i', 4), 'int32': ('i', 4), 'uint': ('I', 4), 'uint32': ('I', 4),
    'float': ('f', 4), 'float32': ('f', 4), 'double': ('d', 8), 'float64': ('d', 8),
}


def _ascii_scalar(token, code):
    if code in ('f', 'd'):
        return float(token)
    value = int(token)
    bits = struct.calcsize(code) * 8
    signed = code.islower()
    if not (-(2**(bits - 1)) if signed else 0) <= value < 2**(bits - (1 if signed else 0)):
        raise ValueError('Integer field exceeds its declared type')
    return value


def _header(data, terminator):
    offset, lines = 0, []
    while offset < min(len(data), _HEADER_BYTES):
        end = data.find(b'\n', offset)
        if end < 0 or end >= _HEADER_BYTES:
            break
        try:
            line = data[offset:end].decode('ascii').strip()
        except UnicodeError as error:
            raise ValueError('Point cloud header must be ASCII') from error
        offset = end + 1
        lines.append(line)
        if terminator(line):
            return lines, data[offset:]
    raise ValueError('Missing or oversized point cloud header')


def _pack_point(cloud, point):
    cloud.extend(struct.pack('<fff', *_coordinates(point)))


def _pcd(data):
    lines, payload = _header(data, lambda line: bool(line.split()) and line.split()[0].upper() == 'DATA')
    header = {}
    for line in lines:
        tokens = line.split('#', 1)[0].split()
        if not tokens:
            continue
        key = tokens[0].upper()
        if key in header:
            raise ValueError('Duplicate PCD header field: ' + key)
        if key not in ('VERSION', 'FIELDS', 'SIZE', 'TYPE', 'COUNT', 'WIDTH', 'HEIGHT', 'VIEWPOINT', 'POINTS', 'DATA'):
            raise ValueError('Unsupported PCD header field: ' + key)
        header[key] = tokens[1:]
    if not {'FIELDS', 'SIZE', 'TYPE', 'WIDTH', 'HEIGHT', 'POINTS', 'DATA'} <= set(header):
        raise ValueError('PCD header is missing required fields')
    names, sizes, types = header['FIELDS'], [int(x) for x in header['SIZE']], header['TYPE']
    counts = [int(x) for x in header.get('COUNT', ['1'] * len(names))]
    if not names or len(names) > 128 or len(set(names)) != len(names) or not len(names) == len(sizes) == len(types) == len(counts):
        raise ValueError('PCD field definitions must be unique and have matching lengths')
    if not all(name in names and counts[names.index(name)] == 1 for name in ('x', 'y', 'z')):
        raise ValueError('PCD requires scalar x, y and z fields')
    if any(value < 1 or value > 1024 for value in counts):
        raise ValueError('PCD COUNT exceeds the supported bound')
    codes = []
    lookup = {('F', 4): 'f', ('F', 8): 'd', ('I', 1): 'b', ('I', 2): 'h', ('I', 4): 'i', ('I', 8): 'q',
              ('U', 1): 'B', ('U', 2): 'H', ('U', 4): 'I', ('U', 8): 'Q'}
    offsets, column = [], 0
    for name, size, kind, count in zip(names, sizes, types, counts):
        if (kind, size) not in lookup:
            raise ValueError('Unsupported PCD scalar type')
        offsets.append(column)
        column += count
        codes.extend([lookup[kind, size]] * count)
    record = struct.Struct('<' + ''.join(codes))
    if record.size > 65536:
        raise ValueError('PCD record exceeds the supported bound')
    for field in ('WIDTH', 'HEIGHT', 'POINTS', 'DATA'):
        if len(header[field]) != 1:
            raise ValueError('PCD ' + field + ' must contain one value')
    width, height, count = [int(header[field][0]) for field in ('WIDTH', 'HEIGHT', 'POINTS')]
    if not 1 <= count <= MAX_POINTS or width < 1 or height < 1 or width * height != count:
        raise ValueError('PCD dimensions and point count must match and stay within the point budget')
    xyz = [offsets[names.index(name)] for name in ('x', 'y', 'z')]
    cloud = bytearray()
    encoding = header['DATA'][0]
    if encoding == 'binary':
        if len(payload) != count * record.size:
            raise ValueError('PCD binary payload length does not match POINTS')
        for values in record.iter_unpack(payload):
            _pack_point(cloud, (values[index] for index in xyz))
    elif encoding == 'ascii':
        seen = 0
        for row in _ascii_rows(payload, comments=True):
            seen += 1
            if seen > count:
                raise ValueError('PCD ASCII row count does not match POINTS')
            if len(row) != len(codes):
                raise ValueError('PCD ASCII row does not match its field declarations')
            values = [_ascii_scalar(token, code) for token, code in zip(row, codes)]
            _pack_point(cloud, (values[index] for index in xyz))
        if seen != count:
            raise ValueError('PCD ASCII row count does not match POINTS')
    else:
        raise ValueError('Unsupported PCD encoding; compressed PCD is not accepted')
    return bytes(cloud), names


def _ascii_rows(payload, comments=False):
    for raw_line in io.BytesIO(payload):
        if len(raw_line) > MAX_SCENE_BYTES:
            raise ValueError('Point cloud ASCII row exceeds the bounded parser budget')
        try:
            line = raw_line.decode('ascii')
        except UnicodeError as error:
            raise ValueError('Point cloud ASCII payload is not ASCII') from error
        row = (line.split('#', 1)[0] if comments else line).split()
        if row:
            yield row


def _ply(data):
    lines, payload = _header(data, lambda line: line == 'end_header')
    if not lines or lines[0] != 'ply':
        raise ValueError('PLY must start with its magic header')
    encoding, elements = None, []
    for line in lines[1:]:
        tokens = line.split()
        if not tokens or tokens[0] in ('comment', 'obj_info', 'end_header'):
            continue
        if tokens[0] == 'format':
            if encoding is not None or len(tokens) != 3 or tokens[2] != '1.0' or tokens[1] not in ('ascii', 'binary_little_endian'):
                raise ValueError('PLY requires ASCII or binary_little_endian format 1.0')
            encoding = tokens[1]
        elif tokens[0] == 'element':
            if len(tokens) != 3 or len(elements) >= 32 or tokens[1] in [item['name'] for item in elements]:
                raise ValueError('Invalid or duplicate PLY element declaration')
            count = int(tokens[2])
            if not 0 <= count <= MAX_POINTS:
                raise ValueError('PLY element count exceeds the supported bound')
            elements.append(dict(name=tokens[1], count=count, properties=[]))
        elif tokens[0] == 'property':
            if not elements or len(elements[-1]['properties']) >= 128:
                raise ValueError('PLY property requires an element and a bounded property count')
            if len(tokens) == 3 and tokens[1] in _SCALARS:
                prop = (tokens[2], _SCALARS[tokens[1]][0], None)
            elif len(tokens) == 5 and tokens[1] == 'list' and tokens[2] in _SCALARS and tokens[3] in _SCALARS and _SCALARS[tokens[2]][0] not in ('f', 'd'):
                prop = (tokens[4], _SCALARS[tokens[3]][0], _SCALARS[tokens[2]][0])
            else:
                raise ValueError('Unsupported PLY property type')
            if prop[0] in [item[0] for item in elements[-1]['properties']]:
                raise ValueError('Duplicate PLY property')
            elements[-1]['properties'].append(prop)
        else:
            raise ValueError('Unsupported PLY header statement')
    vertex = next((item for item in elements if item['name'] == 'vertex'), None)
    if encoding is None or vertex is None or not vertex['count']:
        raise ValueError('PLY requires a nonempty vertex element')
    names = [item[0] for item in vertex['properties']]
    if any(name not in names or vertex['properties'][names.index(name)][2] is not None for name in ('x', 'y', 'z')):
        raise ValueError('PLY requires scalar x, y and z vertex properties')
    if sum(item['count'] for item in elements) > 2 * MAX_POINTS:
        raise ValueError('PLY record count exceeds the bounded input budget')
    cloud, offset = bytearray(), 0
    if encoding == 'ascii':
        row_iter = _ascii_rows(payload)
    for element in elements:
        if not element['properties'] and element['count']:
            raise ValueError('PLY records require properties')
        for _ in range(element['count']):
            values = {}
            if encoding == 'ascii':
                try:
                    row, position = next(row_iter), 0
                except StopIteration as error:
                    raise ValueError('PLY ASCII row count does not match its elements') from error
            for name, code, list_code in element['properties']:
                def take(kind):
                    nonlocal position, offset
                    if encoding == 'ascii':
                        if position >= len(row):
                            raise ValueError('Truncated PLY ASCII property')
                        value = _ascii_scalar(row[position], kind)
                        position += 1
                        return value
                    size = struct.calcsize('<' + kind)
                    if offset + size > len(payload):
                        raise ValueError('Truncated PLY binary property')
                    value = struct.unpack_from('<' + kind, payload, offset)[0]
                    offset += size
                    return value
                if list_code:
                    count = take(list_code)
                    if not 0 <= count <= 65536:
                        raise ValueError('PLY list count exceeds the supported bound')
                    for _ in range(count):
                        take(code)
                else:
                    values[name] = take(code)
            if encoding == 'ascii' and position != len(row):
                raise ValueError('PLY ASCII row contains undeclared values')
            if element['name'] == 'vertex':
                _pack_point(cloud, (values[name] for name in ('x', 'y', 'z')))
    if encoding == 'ascii' and next(row_iter, None) is not None:
        raise ValueError('PLY ASCII row count does not match its elements')
    if encoding != 'ascii' and offset != len(payload):
        raise ValueError('PLY binary payload contains undeclared bytes')
    return bytes(cloud), names


def _import(parameters, input_path, resources, recipe):
    raw = _read(input_path)
    format_name = parameters['format']
    if format_name == 'pcd':
        cloud, fields = _pcd(raw)
    elif format_name == 'ply':
        cloud, fields = _ply(raw)
    else:
        if len(raw) % 12 or len(raw) // 12 > MAX_POINTS:
            raise ValueError('xyz-f32-le requires complete XYZ float32 triplets within the point budget')
        for point in struct.iter_unpack('<fff', raw):
            _coordinates(point)
        cloud, fields = raw, ['x', 'y', 'z']
    original = 'originals/source.' + ('xyz' if format_name == 'xyz-f32-le' else format_name)
    _add_resource(resources, original, raw)
    recipe.update(input=dict(file=original, sha256=_sha(raw), format=format_name, fields=fields),
                  point_count=len(cloud) // 12, point_cloud_role='sensing_source',
                  geometry_status='not_reconstructed', normalization='XYZ float32 little endian; point order and count preserved',
                  resolution=parameters['resolution'], representation=parameters['representation'], frame='world')
    return cloud


def generate_scene(name, source, parameters, *, input_path=None, tree_models=()):
    """Produce one existing-format scene asset without writing or launching.

    ``name`` is a readable scene name; ``source`` names a SOURCE_CATALOG entry.
    Unknown parameters, booleans in numeric fields, noninteger seeds/counts,
    nonfinite values and unbounded requests are rejected with ValueError.
    ``input_path`` and ``tree_models`` are caller-owned local upload paths,
    never browser command text. Import resolution and representation must be
    explicit; no voxelization, downsampling or surface thickening is performed.

    Returns ``scene`` (ordinary xgc2.scene.v1 dict), ``resources`` (relative
    path -> bytes), ``point_cloud`` (imported sensing XYZ bytes or None), and ``recipe``
    (provenance/parameters). Imported clouds are sensing sources; their absence
    of reconstructed obstacle geometry is explicit, including no default plane.
    """
    scene = _scene(name)
    values = _validated(source, parameters)
    if source not in ('pointcloud', 'mockamap') and input_path is not None:
        raise ValueError('input_path is only accepted for point cloud imports')
    if source != 'tree-forest' and tree_models:
        raise ValueError('tree_models are only accepted for tree forests')
    resources = {}
    recipe = dict(source=source, producer='xgc2-platform-scene-sources', producer_version=1,
                  parameters=values, frame='world')
    cloud = None
    if source == 'primitive-forest':
        cloud = _primitive(scene, values, recipe)
    elif source == 'tree-forest':
        cloud = _trees(scene, values, tree_models, resources, recipe)
    else:
        cloud = _import(values, input_path, resources, recipe)
    if cloud is not None:
        _add_resource(resources, 'reference.xyz', cloud)
        recipe['point_cloud'] = dict(file='reference.xyz', sha256=_sha(cloud), format='xyz-f32-le', frame='world')
    if len(json.dumps(scene, allow_nan=False, separators=(',', ':')).encode('utf-8')) > MAX_SCENE_BYTES:
        raise ValueError('Scene document exceeds the existing 1 MiB field limit')
    if any(len(data) > MAX_ATTACHMENT_BYTES for data in resources.values()):
        raise ValueError('Scene attachment exceeds the existing 48 MiB limit')
    if sum(len(data) for data in resources.values()) > MAX_ATTACHMENT_BYTES:
        raise ValueError('Combined scene resources exceed the bounded 48 MiB producer budget')
    recipe['resources'] = [dict(file=file, sha256=_sha(data), bytes=len(data)) for file, data in sorted(resources.items())]
    return dict(scene=scene, resources=resources,
                point_cloud=cloud if source in ('pointcloud', 'mockamap') else None, recipe=recipe)
