"""Resolve a saved scene or one external generator result, once per run file."""
import json
import math
import os
from pathlib import Path
import subprocess
import selectors
import signal
import tempfile
import time

from .document import SceneError, document
from .store import MAX_DOCUMENT_BYTES, SceneLoader, digest, dump_yaml, load
import yaml


def run_generator(argv, output_limit):
    """Bound stdout/stderr before capture and reap the entire owned process group."""
    process = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, start_new_session=True)
    buffers = {'stdout': bytearray(), 'stderr': bytearray()}
    limits = {'stdout': output_limit, 'stderr': 65536}
    selector = selectors.DefaultSelector()
    for name in buffers:
        selector.register(getattr(process, name), selectors.EVENT_READ, name)
    deadline = time.monotonic()+30
    try:
        while selector.get_map():
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise SceneError('Scene generator exceeded its 30 second budget')
            for key, _ in selector.select(remaining):
                chunk = os.read(key.fileobj.fileno(), 65536)
                if not chunk:
                    selector.unregister(key.fileobj)
                elif len(buffers[key.data])+len(chunk) > limits[key.data]:
                    raise SceneError('Scene generator {} exceeds its byte limit'.format(key.data))
                else:
                    buffers[key.data].extend(chunk)
        code = process.wait(timeout=max(.001, deadline-time.monotonic()))
        if code:
            raise SceneError('Scene generator exited with status {}'.format(code))
        return bytes(buffers['stdout'])
    finally:
        # A successful generator can still leave children holding resources.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        selector.close()
        process.stdout.close(); process.stderr.close()
        process.wait()


def resolve(source, working_file='', overrides=None, *, max_bytes=MAX_DOCUMENT_BYTES):
    with Path(source).open('rb') as stream:
        raw = stream.read(MAX_DOCUMENT_BYTES+1)
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
        stdout = run_generator(argv, min(MAX_DOCUMENT_BYTES, max_bytes))
    except (OSError, subprocess.SubprocessError) as error:
        raise SceneError('Scene generator failed: {}'.format(error))
    try:
        produced = json.loads(stdout)
        generated = document(produced['scene'])
    except (ValueError, KeyError, TypeError) as error:
        raise SceneError('Invalid generator result: {}'.format(error))
    target.parent.mkdir(parents=True, exist_ok=True)
    # Preserve the third-party provenance and exact parameters alongside the
    # saved result. All consumers receive the one generated scene snapshot.
    metadata = {key: value for key, value in produced.items() if key != 'scene'}
    metadata.update(parameters=params, sourceSha256=digest(raw), mode='random', format='geometry')
    artifacts = ((target.with_suffix('.generation.yaml'), dump_yaml(metadata)), (target, dump_yaml(generated)))
    if sum(len(body.encode('utf-8')) for _, body in artifacts) > max_bytes:
        raise SceneError('Generated scene artifacts exceed the granted byte limit')
    for path, body in artifacts:
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
