"""Single scene writer with revision checks, bounded undo and atomic YAML saves."""

import copy
import hashlib
import json
import os
import stat
from pathlib import Path
import tempfile
import threading
import time
import uuid
from collections import OrderedDict

import yaml

from .document import SceneError, document, fields, identifier, obstacle
from .motion import state

MAX_DOCUMENT_BYTES = 8 * 1024 * 1024


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if not isinstance(key, str):
            raise SceneError('Scene field names must be strings')
        if key in result:
            raise SceneError('Duplicate scene field {}'.format(key))
        result[key] = value
    return result


class SceneLoader(yaml.SafeLoader):
    pass


def yaml_object(loader, node):
    loader.flatten_mapping(node)
    return unique_object(loader.construct_pairs(node, deep=True))


SceneLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, yaml_object)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def load(path):
    data = Path(path).read_bytes()
    if len(data) > MAX_DOCUMENT_BYTES:
        raise SceneError('Scene file exceeds the supported size')
    try:
        return document(yaml.load(data, Loader=SceneLoader)), digest(data)
    except yaml.YAMLError as error:
        raise SceneError('Invalid scene YAML: {}'.format(error))


class SceneStore:
    def __init__(self, initial, source=None, save_root=None, apply=None, clock=time.monotonic, source_digest=None):
        self.lock = threading.RLock()
        self.document = document(initial)
        self.epoch = str(uuid.uuid4())
        self.revision = 1
        self.next_revision = 2
        self.saved_revision = 1
        self.saved_document = copy.deepcopy(self.document)
        self.source = Path(source).resolve() if source else None
        self.save_root = Path(save_root).resolve() if save_root else (self.source.parent if self.source else None)
        self.file_digests = {}
        if self.source and self.source.exists():
            self.file_digests[self.source] = source_digest or digest(self.source.read_bytes())
        self.apply = apply or (lambda doc, epoch, revision: None)
        self.clock = clock
        self.playing = False
        self.elapsed = 0.0
        self.started_at = clock()
        self.undo_stack = []
        self.redo_stack = []
        self.requests = OrderedDict()
        self.apply(self.document, self.epoch, self.revision)

    def scene_time(self):
        return self.elapsed + (max(0.0, self.clock()-self.started_at) if self.playing else 0.0)

    def envelope(self):
        with self.lock:
            return {'epoch': self.epoch, 'revision': self.revision, 'savedRevision': self.saved_revision,
                    'dirty': self.document != self.saved_document, 'playing': self.playing,
                    'sceneTime': self.scene_time(), 'document': copy.deepcopy(self.document)}

    def states(self):
        with self.lock:
            elapsed = self.scene_time()
            return [state(item, elapsed, self.playing) for item in self.document['obstacles']]

    def command(self, request):
        with self.lock:
            try:
                fields(request, ('requestId', 'expectedEpoch', 'expectedRevision', 'operation', 'obstacle', 'id', 'document', 'path'), ('operation',))
                if request['operation'] == 'get':
                    return dict(self.envelope(), success=True)
                rid = identifier(request.get('requestId'), 'Request ID')
                fingerprint = json.dumps(request, sort_keys=True, allow_nan=False)
                previous = self.requests.get(rid)
                if previous:
                    if previous[0] != fingerprint:
                        raise SceneError('Request ID was already used for a different operation')
                    return copy.deepcopy(previous[1])
                if request.get('expectedEpoch') != self.epoch or type(request.get('expectedRevision')) is not int or request['expectedRevision'] != self.revision:
                    raise SceneError('Scene changed; refresh before editing')
                self._execute(request)
                result = dict(self.envelope(), success=True)
                self.requests[rid] = (fingerprint, result)
                while len(self.requests) > 256:
                    self.requests.popitem(last=False)
                return copy.deepcopy(result)
            except (SceneError, OSError, ValueError, TypeError) as error:
                return dict(self.envelope(), success=False, error=str(error))

    def _execute(self, request):
        operation = request['operation']
        if operation == 'resync':
            self.revision = self._apply(self.document)
            return
        if operation == 'save':
            self._save(request.get('path'))
            return
        if operation in ('play', 'pause', 'reset'):
            elapsed = self.scene_time()
            self.playing = operation == 'play'
            self.elapsed = 0.0 if operation == 'reset' else elapsed
            self.started_at = self.clock()
            return
        if operation in ('undo', 'redo'):
            source = self.undo_stack if operation == 'undo' else self.redo_stack
            target = self.redo_stack if operation == 'undo' else self.undo_stack
            if not source:
                raise SceneError('Nothing to {}'.format(operation))
            replacement = source[-1]
            revision = self._apply(replacement)
            target.append(self.document)
            source.pop()
            self.document = replacement
            self.revision = revision
            return
        candidate = copy.deepcopy(self.document)
        obstacles = candidate['obstacles']
        if operation in ('add', 'update'):
            replacement = obstacle(request.get('obstacle'))
            indices = [i for i, item in enumerate(obstacles) if item['id'] == replacement['id']]
            if operation == 'add':
                if indices:
                    raise SceneError('Obstacle ID already exists')
                obstacles.append(replacement)
            else:
                if not indices:
                    raise SceneError('Obstacle no longer exists')
                obstacles[indices[0]] = replacement
        elif operation == 'delete':
            oid = identifier(request.get('id'), 'Obstacle ID')
            candidate['obstacles'] = [item for item in obstacles if item['id'] != oid]
            if len(candidate['obstacles']) == len(obstacles):
                raise SceneError('Obstacle no longer exists')
        elif operation == 'clear':
            candidate['obstacles'] = []
        elif operation == 'replace':
            candidate = document(request.get('document'))
        else:
            raise SceneError('Unsupported scene operation {!r}'.format(operation))
        candidate = document(candidate)
        if candidate == self.document:
            return
        revision = self._apply(candidate)
        self.undo_stack.append(self.document)
        self.undo_stack = self.undo_stack[-128:]
        self.redo_stack.clear()
        self.document = candidate
        self.revision = revision

    def _apply(self, candidate):
        # Reserve the identity before transport. A lost reply may mean the
        # simulator applied this candidate, so a different candidate must never
        # reuse its revision. Resync can restore the accepted document at a new
        # revision after either a partial failure or an ambiguous lost reply.
        revision = self.next_revision
        self.next_revision += 1
        self.apply(candidate, self.epoch, revision)
        return revision

    def _save(self, relative):
        if self.save_root is None:
            raise SceneError('This scene has no writable project directory configured')
        if relative is not None:
            if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
                raise SceneError('Save path must be relative to the configured project directory')
            target = (self.save_root/relative).resolve()
        else:
            target = self.source
        if target is None:
            raise SceneError('Choose a scene YAML filename')
        try:
            target.relative_to(self.save_root)
        except ValueError:
            raise SceneError('Save target is outside the configured project directory')
        if target.suffix.lower() not in ('.yaml', '.yml'):
            raise SceneError('Save target must be a YAML file')
        if not target.parent.is_dir():
            raise SceneError('Save directory does not exist')
        current = digest(target.read_bytes()) if target.exists() else None
        expected = self.file_digests.get(target)
        if current != expected:
            raise SceneError('Scene file changed outside the editor; reload or save to another filename')
        encoded = yaml.safe_dump(self.document, allow_unicode=True, sort_keys=False).encode('utf-8')
        handle, temporary = tempfile.mkstemp(prefix='.'+target.name+'.', suffix='.tmp', dir=str(target.parent))
        try:
            with os.fdopen(handle, 'wb') as stream:
                if target.exists():
                    os.fchmod(stream.fileno(), stat.S_IMODE(target.stat().st_mode))
                stream.write(encoded)
                stream.flush()
                os.fsync(stream.fileno())
            # Recheck after serialization; do not overwrite an external edit detected here.
            actual = digest(target.read_bytes()) if target.exists() else None
            if actual != expected:
                raise SceneError('Scene file changed while saving')
            os.replace(temporary, str(target))
            directory = os.open(str(target.parent), os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        self.source = target
        self.file_digests[target] = digest(encoded)
        self.saved_document = copy.deepcopy(self.document)
        self.saved_revision = self.revision
