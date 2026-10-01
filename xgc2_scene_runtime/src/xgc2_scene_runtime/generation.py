"""Resolve a saved scene or one external generator result, once per run file."""
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile

from .document import SceneError, document
from .store import MAX_DOCUMENT_BYTES, SceneLoader, digest, dump_yaml, load
import yaml


def resolve(source, working_file='', overrides=None):
    raw = Path(source).read_bytes()
    if len(raw) > MAX_DOCUMENT_BYTES:
        raise SceneError('Scene source exceeds the supported size')
    value = yaml.load(raw, Loader=SceneLoader)
    if not isinstance(value, dict) or value.get('schema') != 'xgc2.scene-source.v1':
        return load(source), source
    if value.get('format') != 'geometry' or value.get('mode') not in ('fixed', 'random'):
        raise SceneError('Scene source requires explicit geometry format and fixed/random mode')
    if value['mode'] == 'fixed':
        return (document(value['document']), digest(raw)), source
    if not working_file:
        raise SceneError('Random scene requires a run-owned result file')
    target = Path(working_file)
    if target.is_file():
        return load(str(target)), str(target)
    generator = value['generator']
    command = generator.get('command')
    if not isinstance(command, list) or not command or any(not isinstance(x, str) or '\x00' in x for x in command):
        raise SceneError('Scene generator requires an explicit argument vector')
    params = dict(generator.get('parameters', {}))
    requested = overrides or {}
    if not isinstance(requested, dict) or set(requested) - set(params):
        raise SceneError('Scene generation parameters are not declared by this generator')
    params.update(requested)
    argv = list(command)
    for key, value in params.items():
        if not isinstance(key, str) or not key or key.startswith('-') or not all(c.isalnum() or c == '-' for c in key):
            raise SceneError('Invalid generator parameter name')
        if type(value) not in (int, float) or not math.isfinite(value):
            raise SceneError('Generator parameter must be a finite number')
        argv.extend(['--' + key, str(value)])
    try:
        result = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=30, check=True)
    except (OSError, subprocess.SubprocessError) as error:
        raise SceneError('Scene generator failed: {}'.format(error))
    if len(result.stdout) > MAX_DOCUMENT_BYTES:
        raise SceneError('Generated scene exceeds the supported size')
    try:
        produced = json.loads(result.stdout)
        generated = document(produced['scene'])
    except (ValueError, KeyError, TypeError) as error:
        raise SceneError('Invalid generator result: {}'.format(error))
    target.parent.mkdir(parents=True, exist_ok=True)
    # Preserve the third-party provenance and exact parameters alongside the
    # saved result. All consumers receive the one generated scene snapshot.
    metadata = {key: value for key, value in produced.items() if key != 'scene'}
    metadata.update(parameters=params, sourceSha256=digest(raw), mode='random', format='geometry')
    for path, body in ((target.with_suffix('.generation.yaml'), dump_yaml(metadata)), (target, dump_yaml(generated))):
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=str(path.parent), delete=False) as handle:
                temporary = handle.name
                handle.write(body)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, str(path))
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
    return load(str(target)), str(target)
