#!/usr/bin/env python3
"""Prepare a selected scene's immutable inputs and provider-owned working copy."""
import argparse
import json
import os
from pathlib import Path
import re
import sys

from xgc2_scene_runtime.document import SceneError, fields
from xgc2_scene_runtime.generation import resolve
from xgc2_scene_runtime.prepare import DEFAULT_WORLD, attachment, checked_path, parameters, prepare, read
from xgc2_scene_runtime.store import unique_object


def inputs(context, resource_root, workspace_root):
    selection = fields(context['scene'], ('asset', 'simulator', 'parameters'), ('asset', 'simulator'))
    name, simulator = selection['asset'], selection['simulator']
    if (not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9 _-]{0,79}', name)
            or name.strip() != name or '..' in name):
        raise SceneError('Invalid scene asset name')
    root = Path(resource_root)
    if not root.is_absolute() or not root.is_dir():
        raise SceneError('An explicit existing resource root is required')
    root = root.resolve(strict=True)
    directory = checked_path(str(root/'Shared'/'Scenes'/name), root/'Shared'/'Scenes', directory=True)
    manifest = json.loads(read(directory/'manifest.json', 65536), object_pairs_hook=unique_object)
    if type(manifest.get('schemaVersion')) is not int or manifest['schemaVersion'] != 1 or manifest.get('kind') != 'xgc.scene-replay-asset.v1':
        raise SceneError('Unsupported scene asset manifest schema')
    supports = manifest.get('simulators')
    if not isinstance(supports, dict) or simulator not in ('gazebo', 'xsim') or not isinstance(supports.get(simulator), dict):
        raise SceneError('The selected simulator is not supported by the asset')
    mode = supports[simulator].get('geometry')
    if mode not in ('document', 'native') or (simulator == 'xsim' and mode != 'document'):
        raise SceneError('Unsupported geometry for the selected simulator')
    scene = attachment(directory, manifest.get('sceneDocument'), required=mode == 'document')
    world = attachment(directory, manifest.get('gazeboWorld'), required=simulator == 'gazebo' and mode == 'native') if simulator == 'gazebo' else ''
    template = simulator == 'gazebo' and not world
    if template:
        world = str(DEFAULT_WORLD)
        if not DEFAULT_WORLD.is_file():
            raise SceneError('The provider editable-world template is not installed')
    authored = selection.get('parameters', {})
    if not isinstance(authored, dict):
        raise SceneError('Scene parameters must be an object')
    physics = parameters(authored, template=template) if simulator == 'gazebo' else authored
    random = (manifest.get('obstacleInput') or {}).get('mode') == 'random'
    editable = mode == 'document' and all(isinstance(v, dict) and v.get('geometry') == 'document' for v in supports.values())
    working = ''
    experiment = context.get('experimentResourceId', '')
    if (editable or random) and experiment:
        if not isinstance(experiment, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', experiment):
            raise SceneError('A canonical experiment resource identity is required')
        grant = Path(workspace_root)
        if not grant.is_absolute() or not grant.is_dir() or grant.is_symlink():
            raise SceneError('An existing provider workspace grant is required')
        output = grant.resolve(strict=True)/'native-scenes'/experiment/name
        if random:
            run = context.get('openingRunId', '')
            if not isinstance(run, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', run):
                raise SceneError('Random geometry requires the actual Session opening Run identity')
            output = output/run
        # Every created child stays inside the explicit owner grant; never follow
        # a replacement symlink into another product's files.
        relative = output.relative_to(grant.resolve(strict=True))
        current = grant.resolve(strict=True)
        for part in relative.parts:
            current = current/part
            if current.is_symlink():
                raise SceneError('Provider workspace cannot contain symlinks')
            current.mkdir(mode=0o750, exist_ok=True)
        working = str(output/'scene-document.yaml')
    elif random:
        raise SceneError('Random geometry requires an Experiment workspace')
    if scene and random:
        (_, _), scene = resolve(scene, working, authored.get('generation', {}), max_bytes=8*1024*1024)
    value = dict(assetDirectory=str(directory), simulator=simulator, sceneFile=scene, worldFile=world,
                 worldAttachment=bool(manifest.get('gazeboWorld')) and simulator == 'gazebo',
                 projectGeometry=simulator == 'gazebo' and mode == 'document', workingFile=working,
                 frozen=not bool(working), parameters=physics,
                 generationParametersJson=json.dumps(authored.get('generation', {}), allow_nan=False, separators=(',', ':')))
    for key, source in (('mediaFile', 'media'), ('cameraInfoFile', 'cameraInfo'), ('extrinsicFile', 'extrinsic')):
        value[key] = attachment(directory, manifest.get(source))
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--context-json', required=True)
    parser.add_argument('--resource-root', default=os.environ.get('XGC_USER_FILES_DIR', ''))
    parser.add_argument('--workspace-root', default=os.environ.get('XGC_JOB_ARTIFACT_ROOT', ''))
    parser.add_argument('--prepare-world', action='store_true')
    parser.add_argument('--socket-path', default='')
    parser.add_argument('--target-id', default='')
    parser.add_argument('--configuration-revision', type=int, default=None)
    args = parser.parse_args()
    try:
        if len(args.context_json.encode('utf-8')) > 65536:
            raise SceneError('Scene context exceeds 64 KiB')
        context = json.loads(args.context_json, object_pairs_hook=unique_object,
                             parse_constant=lambda _: (_ for _ in ()).throw(SceneError('Nonfinite JSON')))
        value = inputs(context, args.resource_root, args.workspace_root)
        if args.prepare_world and value['simulator'] == 'gazebo':
            run = context.get('openingRunId', '')
            if not isinstance(run, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', run):
                raise SceneError('World materialization requires the actual Session opening Run identity')
            grant = Path(args.workspace_root)
            if not grant.is_absolute() or not grant.is_dir() or grant.is_symlink():
                raise SceneError('An existing provider workspace grant is required')
            output = Path(value['workingFile']).parent if value['workingFile'] else grant/'native-worlds'/run
            current = grant.resolve(strict=True)
            for part in output.relative_to(current).parts:
                current = current/part
                if current.is_symlink(): raise SceneError('Provider workspace cannot contain symlinks')
                current.mkdir(mode=0o750, exist_ok=True)
            receipt = prepare({'schema':'xgc2.simulation.prepare.v1',
                               'configuration_revision':args.configuration_revision,
                               'resource_root':args.resource_root, 'asset_directory':value['assetDirectory'],
                               'socket_path':args.socket_path, 'target_id':args.target_id,
                               'output_grant':{'directory':str(output), 'max_bytes':16*1024*1024},
                               'parameters':value['parameters'], 'editable':not value['frozen'], 'run_id':run})
            value.update(receipt)
        print(json.dumps(value, allow_nan=False, separators=(',', ':')))
    except (OSError, ValueError, KeyError, TypeError) as error:
        print('scene_prepare: '+str(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
