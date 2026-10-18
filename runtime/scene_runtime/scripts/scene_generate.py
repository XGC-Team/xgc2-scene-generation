#!/usr/bin/env python3
"""Produce one ordinary scene upload from an installed scene source."""

import argparse
import base64
import json
import math
import sys
import tempfile
from pathlib import Path

from xgc2_scene_runtime.scene_sources import SOURCE_CATALOG, generate_scene


MAX_INPUT = 64 * 1024 * 1024
MAX_FILE = 48 * 1024 * 1024


def upload(value, directory, used):
    if not isinstance(value, dict) or set(value) != {'file', 'base64'}:
        raise ValueError('an upload requires file and base64')
    name = value['file']
    if (not isinstance(name, str) or not name or name.startswith('.')
            or '/' in name or '\\' in name or '\0' in name or name in used):
        raise ValueError('upload names must be distinct plain file names')
    try:
        data = base64.b64decode(value['base64'], validate=True)
    except (TypeError, ValueError) as error:
        raise ValueError('invalid uploaded file base64') from error
    if not data or len(data) > MAX_FILE:
        raise ValueError('uploaded file must be nonempty and at most 48 MiB')
    used.add(name)
    target = directory / name
    target.write_bytes(data)
    return target


def produce(request):
    if not isinstance(request, dict) or set(request) - {'name', 'source', 'parameters', 'input', 'treeModels'}:
        raise ValueError('invalid scene generation request')
    source, name = request.get('source'), request.get('name')
    if source not in {item['id'] for item in SOURCE_CATALOG}:
        raise ValueError('unknown scene source')
    parameters = request.get('parameters', {})
    models = request.get('treeModels', [])
    if not isinstance(models, list) or len(models) > 32:
        raise ValueError('at most 32 tree model uploads are supported')
    if request.get('input') is not None and source not in ('pointcloud', 'mockamap'):
        raise ValueError('this scene source does not accept a point cloud upload')
    if models and source != 'tree-forest':
        raise ValueError('this scene source does not accept tree model uploads')
    with tempfile.TemporaryDirectory(prefix='xgc-scene-source-') as temporary:
        directory, used = Path(temporary), set()
        input_path = upload(request['input'], directory, used) if request.get('input') is not None else None
        trees = [upload(model, directory, used) for model in models]
        result = generate_scene(name, source, parameters, input_path=input_path, tree_models=trees)
    resources = dict(result['resources'])
    resources['generation.json'] = (json.dumps(result['recipe'], sort_keys=True, indent=2, allow_nan=False) + '\n').encode()
    cloud = result.get('point_cloud')
    # The fixed point cloud has its own manifest role. Do not upload the same
    # path twice through resources and pointCloud.
    if cloud is not None:
        resources.pop('reference.xyz', None)
    if any(len(data) > MAX_FILE for data in resources.values()) or sum(map(len, resources.values())) + len(cloud or b'') > 90 * 1024 * 1024:
        raise ValueError('generated scene resources exceed the upload budget')
    document = json.dumps(result['scene'], separators=(',', ':'), allow_nan=False)
    if len(document.encode()) > 1024 * 1024:
        raise ValueError('generated scene document exceeds 1 MiB')
    numeric = {key: value for key, value in parameters.items()
               if type(value) in (int, float) and math.isfinite(value)}
    payload = {
        'name': name, 'origin': 'user',
        'note': 'Saved scene from ' + source + '.',
        'sceneDocument': document,
        'obstacleInput': {'mode': 'fixed', 'format': 'pointcloud' if cloud is not None else 'geometry',
                          'generator': source, 'parameters': numeric},
        'resources': [{'file': file, 'base64': base64.b64encode(data).decode('ascii')}
                      for file, data in sorted(resources.items())],
        'simulators': {'xsim': {'geometry': 'document'}},
    }
    if cloud is not None:
        payload['pointCloud'] = {'file': 'reference.xyz', 'format': 'xyz-f32-le', 'frame': 'world',
                                 'base64': base64.b64encode(cloud).decode('ascii')}
    return payload


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--describe', action='store_true')
    args = parser.parse_args()
    try:
        if args.describe:
            result = SOURCE_CATALOG
        else:
            raw = sys.stdin.buffer.read(MAX_INPUT + 1)
            if len(raw) > MAX_INPUT:
                raise ValueError('scene generation input exceeds 64 MiB')
            result = produce(json.loads(raw))
        print(json.dumps(result, separators=(',', ':'), allow_nan=False))
        return 0
    except (ValueError, TypeError, KeyError, OSError, OverflowError) as error:
        # The finite CLI's stdout is its bounded result/error channel.
        print(str(error))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
