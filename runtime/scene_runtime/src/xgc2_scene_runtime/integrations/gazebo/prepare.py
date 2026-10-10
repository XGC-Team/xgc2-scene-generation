"""Gazebo-owned asset interpretation and explicit bounded world materialization."""
import hashlib
import json
import math
import os
import re
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ...document import SceneError, fields
from ...generation import resolve
from ...store import MAX_DOCUMENT_BYTES, dump_yaml, unique_object

SCHEMA = 'xgc2.simulation.prepare.v1'
DEFAULT_WORLD = Path('/opt/ros/noetic/share/gazebo_sim_worlds/worlds/scene_editable/scene_editable.world')


def read(path, limit):
    with Path(path).open('rb') as stream:
        data = stream.read(limit+1)
    if not data or len(data) > limit:
        raise SceneError('Input {} exceeds its {} byte limit or is empty'.format(path, limit))
    return data


def checked_path(value, root, *, directory=False):
    if not isinstance(value, str) or not Path(value).is_absolute():
        raise SceneError('An absolute provider resource path is required')
    path = Path(value).resolve(strict=True)
    try:
        path.relative_to(root)
    except ValueError as error:
        raise SceneError('Resource path is outside the granted root') from error
    if directory and not path.is_dir():
        raise SceneError('Resource directory is required')
    if not directory and not path.is_file():
        raise SceneError('Resource file is required')
    return path


def sha256(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(65536), b''):
            result.update(block)
    return result.hexdigest()


def attachment(directory, value, *, required=False):
    if value is None or value == {}:
        if required:
            raise SceneError('Required scene attachment is absent')
        return ''
    if not isinstance(value, dict):
        raise SceneError('Scene attachment must be an object')
    filename = value.get('file')
    if not isinstance(filename, str) or not filename or Path(filename).name != filename or filename.startswith('.') or '..' in filename:
        raise SceneError('Scene attachment must use a plain asset filename')
    path = checked_path(str(directory/filename), directory)
    expected = value.get('sha256')
    if not isinstance(expected, str) or len(expected) != 64 or any(c not in '0123456789abcdef' for c in expected):
        raise SceneError('Scene attachment requires its exact SHA256')
    if sha256(path) != expected:
        raise SceneError('Scene attachment content does not match its manifest SHA256')
    return str(path)


def parameters(value, *, template):
    fields(value, ('overrideWorldPhysicsTiming', 'maxStepSize', 'realTimeUpdateRate', 'generation', 'chassis_robot_ids', 'required_components', 'clock_rate_hz'))
    result = dict(overrideWorldPhysicsTiming=template, maxStepSize=.004, realTimeUpdateRate=250)
    result.update(value)
    rate = result.get('clock_rate_hz', 0)
    if type(rate) not in (int, float) or not math.isfinite(rate) or not 0 <= rate <= 1e6:
        raise SceneError('clock_rate_hz must be finite in [0, 1000000]')
    result['clock_rate_hz'] = rate
    if type(result['overrideWorldPhysicsTiming']) is not bool:
        raise SceneError('overrideWorldPhysicsTiming must be a boolean')
    for name, positive in (('maxStepSize', True), ('realTimeUpdateRate', False)):
        number = result[name]
        if type(number) not in (float, int) or not math.isfinite(number) or (number <= 0 if positive else number < 0):
            raise SceneError('{} must be a finite {} number'.format(name, 'positive' if positive else 'nonnegative'))
    generation = result.get('generation', {})
    if not isinstance(generation, dict) or any(type(v) not in (float, int) or not math.isfinite(v) for v in generation.values()):
        raise SceneError('generation must contain finite numeric parameters')
    for name in ('chassis_robot_ids', 'required_components'):
        roster = result.get(name, [])
        if (not isinstance(roster, list) or len(roster) > 16 or
                any(not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', identity, re.ASCII)
                    for identity in roster) or len(set(roster)) != len(roster)):
            raise SceneError('{} must contain at most 16 unique ASCII short identifiers'.format(name))
        result[name] = roster
    return result


def atomic(path, data):
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=str(path.parent), prefix='.'+path.name+'.', delete=False) as stream:
            temporary = stream.name
            stream.write(data); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, str(path))
        directory = os.open(str(path.parent), os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)


def prepare(request):
    fields(request, ('schema', 'configuration_revision', 'asset_directory', 'world_file', 'resource_root',
                     'socket_path', 'target_id', 'output_grant', 'parameters', 'editable', 'run_id',
                     'expected_revision_id'), ('schema', 'configuration_revision', 'resource_root',
                     'socket_path', 'target_id', 'output_grant'))
    if request['schema'] != SCHEMA or type(request['configuration_revision']) is not int or not 1 <= request['configuration_revision'] <= 2**53-1:
        raise SceneError('A versioned simulation prepare request is required')
    if bool(request.get('asset_directory')) == bool(request.get('world_file')):
        raise SceneError('Exactly one asset_directory or world_file is required')
    root = Path(request['resource_root'])
    if not root.is_absolute() or not root.is_dir():
        raise SceneError('An existing absolute resource_root is required')
    root = root.resolve(strict=True)
    grant = fields(request['output_grant'], ('directory', 'max_bytes'), ('directory', 'max_bytes'))
    output = Path(grant['directory'])
    if not output.is_absolute() or not output.is_dir() or output.is_symlink():
        raise SceneError('The output grant must identify an existing absolute directory')
    output = output.resolve(strict=True)
    if type(grant['max_bytes']) is not int or not 1 <= grant['max_bytes'] <= 64*1024*1024:
        raise SceneError('The output grant requires a byte limit of at most 64 MiB')
    socket = request['socket_path']
    if not isinstance(socket, str) or not Path(socket).is_absolute() or os.path.normpath(socket) != socket or len(os.fsencode(socket)) > 107:
        raise SceneError('A canonical absolute world socket path is required')
    target = request['target_id']
    if not isinstance(target, str) or not target.strip() or target.strip() != target or len(target) > 128:
        raise SceneError('An explicit target_id is required')
    if type(request.get('editable', False)) is not bool:
        raise SceneError('editable must be a boolean')
    manifest, scene_file, media_file, camera_file, extrinsic_file = {}, '', '', '', ''
    project_geometry, frozen, template = False, True, False
    asset_directory = ''
    if request.get('asset_directory'):
        directory = checked_path(request['asset_directory'], root, directory=True)
        asset_directory = str(directory)
        manifest = json.loads(read(directory/'manifest.json', 65536), object_pairs_hook=unique_object)
        if type(manifest.get('schemaVersion')) is not int or manifest['schemaVersion'] != 1 or manifest.get('kind') != 'xgc.scene-replay-asset.v1':
            raise SceneError('Unsupported scene asset manifest schema')
        if request.get('expected_revision_id') is not None and manifest.get('revisionId') != request['expected_revision_id']:
            raise SceneError('Scene asset revision conflicts with the requested reference')
        simulators = manifest.get('simulators', {})
        if not isinstance(simulators, dict) or not isinstance(simulators.get('gazebo'), dict):
            raise SceneError('The asset must declare Gazebo geometry support')
        mode = simulators['gazebo'].get('geometry')
        if mode not in ('document', 'native'):
            raise SceneError('Gazebo geometry must be document or native')
        project_geometry = mode == 'document'
        scene_file = attachment(directory, manifest.get('sceneDocument'), required=project_geometry)
        if manifest.get('gazeboWorld'):
            world = Path(attachment(directory, manifest['gazeboWorld'], required=True))
        elif mode == 'document':
            world = DEFAULT_WORLD
            if not world.is_file():
                raise SceneError('The provider editable-world template is not installed')
            template = True
        else:
            raise SceneError('Native geometry requires the original Gazebo world attachment')
        frozen = not (project_geometry and request.get('editable', False) and
                      all(isinstance(support, dict) and support.get('geometry') == 'document' for support in simulators.values()))
        media_file = attachment(directory, manifest.get('media'))
        camera_file = attachment(directory, manifest.get('cameraInfo'))
        extrinsic_file = attachment(directory, manifest.get('extrinsic'))
    else:
        world = checked_path(request['world_file'], root)
    physics = parameters(request.get('parameters', {}), template=template)
    raw = read(world, MAX_DOCUMENT_BYTES)
    if b'<!DOCTYPE' in raw.upper() or b'<!ENTITY' in raw.upper():
        raise SceneError('World assets cannot declare XML entities')
    try:
        sdf = ET.fromstring(raw)
    except ET.ParseError as error:
        raise SceneError('Invalid Gazebo world XML') from error
    worlds = sdf.findall('world')
    if sdf.tag != 'sdf' or len(worlds) != 1:
        raise SceneError('A single-world SDF asset is required')
    live = worlds[0]
    for plugin in live.findall('plugin'):
        if plugin.get('filename') in ('libxgc2_simulation_world.so', 'libxgc2_scene_authoring_world.so'):
            raise SceneError('The input world already embeds a platform control owner')
    if physics['overrideWorldPhysicsTiming']:
        element = live.find('physics')
        if element is None:
            element = ET.SubElement(live, 'physics', {'name': 'simulation_physics', 'type': 'ode'})
        for tag, value in (('max_step_size', physics['maxStepSize']), ('real_time_update_rate', physics['realTimeUpdateRate'])):
            item = element.find(tag)
            if item is None: item = ET.SubElement(element, tag)
            item.text = str(value)
    plugin = ET.SubElement(live, 'plugin', {'name': 'xgc2_simulation', 'filename': 'libxgc2_simulation_world.so'})
    for tag, value in (('socket_path', socket), ('target_id', target), ('resource_root', str(root))):
        ET.SubElement(plugin, tag).text = value
    ET.SubElement(plugin, 'configuration_revision').text = str(request['configuration_revision'])
    for identity in physics['chassis_robot_ids']:
        ET.SubElement(plugin, 'chassis_robot_id').text = identity
    for identity in physics['required_components']:
        ET.SubElement(plugin, 'required_component').text = identity
    ET.SubElement(plugin, 'clock_rate_hz').text = str(physics['clock_rate_hz'])
    encoded = ET.tostring(sdf, encoding='utf-8', xml_declaration=True)
    files = {output/'prepared-world.world': encoded}
    working_file = ''
    if project_geometry:
        obstacle_input = manifest.get('obstacleInput') or {}
        if obstacle_input.get('mode') == 'random' and not request.get('run_id'):
            raise SceneError('Random geometry requires an explicit run identity')
        working_file = str(output/'scene-document.yaml') if not frozen or obstacle_input.get('mode') == 'random' else ''
        remaining = grant['max_bytes']-len(encoded)
        if remaining <= 0:
            raise SceneError('Prepared world exhausts the output grant')
        # An uncommitted generator result remains in our temporary grant area;
        # failed validation never replaces a prior prepared world or scene.
        random = obstacle_input.get('mode') == 'random'
        if random and not Path(working_file).exists():
            with tempfile.TemporaryDirectory(prefix='.scene-prepare-', dir=str(output)) as stage:
                stage_file = str(Path(stage)/'scene-document.yaml')
                (document, _), _ = resolve(scene_file, stage_file, physics.get('generation', {}), max_bytes=remaining)
                for path in Path(stage).iterdir():
                    files[output/path.name] = read(path, remaining)
            resolved = working_file
        else:
            (document, _), resolved = resolve(scene_file, working_file, physics.get('generation', {}), max_bytes=remaining)
        if len(document['obstacles']) > 256:
            raise SceneError('This Gazebo world supports at most 256 scene entities')
        if not frozen and resolved == scene_file:
            files[Path(working_file)] = dump_yaml(document).encode('utf-8')
            scene_file = working_file
        else:
            scene_file = resolved
    if sum(len(value) for value in files.values()) > grant['max_bytes']:
        raise SceneError('Prepared artifacts exceed the output grant')
    receipt = {'schema': SCHEMA, 'configuration_revision': request['configuration_revision'],
               'worldFile': str(output/'prepared-world.world'), 'sourceWorldSha256': hashlib.sha256(raw).hexdigest(),
               'preparedWorldSha256': hashlib.sha256(encoded).hexdigest(), 'assetDirectory': asset_directory,
               'sceneFile': scene_file, 'workingFile': working_file, 'frozen': frozen, 'projectGeometry': project_geometry,
               'mediaFile': media_file, 'cameraInfoFile': camera_file, 'extrinsicFile': extrinsic_file,
               'parameters': physics, 'rosData': {'clockRateHz': physics['clock_rate_hz']},
               'writes': [str(path) for path in files]}
    for path, value in files.items():
        atomic(path, value)
    return receipt
