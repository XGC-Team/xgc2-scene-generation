"""Scene-time motion independent of algorithm execution."""

import math
from .document import dot


def multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def rotate(q, value):
    return multiply(multiply(q, list(value)+[0.0]), [-q[0], -q[1], -q[2], q[3]])[:3]


def compose(a, b):
    delta = rotate(a['orientation'], b['position'])
    return {'position': [x+y for x, y in zip(a['position'], delta)],
            'orientation': multiply(a['orientation'], b['orientation'])}


def state(obstacle, elapsed, playing):
    initial = obstacle['pose']
    position = list(initial['position'])
    orientation = list(initial['orientation'])
    spec = obstacle['motion']
    linear, angular = [0.0]*3, [0.0]*3
    kind = spec['type']
    if kind == 'constant_twist':
        linear, angular = list(spec['linear']), list(spec['angular'])
        position = [x+elapsed*v for x, v in zip(position, linear)]
        norm = math.sqrt(dot(angular, angular))
        if norm > 1e-12:
            half = 0.5*norm*elapsed
            rotation = [v/norm*math.sin(half) for v in angular]+[math.cos(half)]
            orientation = multiply(rotation, orientation)
    elif kind == 'ping_pong':
        delta = [b-a for a, b in zip(spec['point_a'], spec['point_b'])]
        length = math.sqrt(dot(delta, delta))
        phase = (elapsed*spec['speed']/length) % 2.0
        fraction, direction = (phase, 1) if phase < 1 else (2-phase, -1)
        position = [a+fraction*d for a, d in zip(spec['point_a'], delta)]
        linear = [direction*spec['speed']*d/length for d in delta]
    elif kind == 'circle':
        phase = spec['phase'] + elapsed*spec['angular_speed']
        radius, speed = spec['radius'], spec['angular_speed']
        position = [spec['center'][0]+radius*math.cos(phase), spec['center'][1]+radius*math.sin(phase), spec['center'][2]]
        linear = [-radius*speed*math.sin(phase), radius*speed*math.cos(phase), 0.0]
    if not playing:
        linear, angular = [0.0]*3, [0.0]*3
    return {'id': obstacle['id'], 'pose': {'position': position, 'orientation': orientation},
            'linear': linear, 'angular': angular}
