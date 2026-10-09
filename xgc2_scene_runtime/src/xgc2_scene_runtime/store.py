"""Single scene writer with revision checks, bounded undo and atomic YAML saves.

Documents are immutable values. Every change builds a new top-level dict and
obstacle list and shares the unchanged obstacles, so undo history, the saved
baseline and command results reference documents instead of copying them.
Nothing may modify a document, its obstacles or their fields in place.
"""

import hashlib
import json
import os
import re
import stat
from pathlib import Path
import tempfile
import threading
import time
import uuid
from collections import OrderedDict

import yaml

from .document import SceneError, check_obstacle_set, document, fields, identifier, obstacle
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


# libyaml parses and emits several times faster than the pure-Python classes;
# a 500-obstacle autosave spent most of an edit in the Python emitter.
_SAFE_LOADER = getattr(yaml, 'CSafeLoader', yaml.SafeLoader)
_SAFE_DUMPER = getattr(yaml, 'CSafeDumper', yaml.SafeDumper)


class SceneLoader(_SAFE_LOADER):
    pass


def dump_yaml(value):
    return yaml.dump(value, Dumper=_SAFE_DUMPER, allow_unicode=True, sort_keys=False)


class DocumentYaml:
    """YAML text of a document, re-emitting only obstacles that changed.

    Obstacles are immutable and shared between revisions, so the text of an
    unchanged obstacle object is reused. The output equals dump_yaml(document):
    the obstacle sequence is the last key and PyYAML writes it unindented, so
    each item is the block that a one-item list dumps to.
    """

    def __init__(self):
        self._items = {}

    def dump(self, value):
        obstacles = value['obstacles']
        if not obstacles or list(value)[-1] != 'obstacles':
            self._items = {}
            return dump_yaml(value)
        items, texts = {}, []
        for item in obstacles:
            cached = self._items.get(id(item))
            if cached is None or cached[0] is not item:
                cached = (item, dump_yaml([item]))
            items[id(item)] = cached
            texts.append(cached[1])
        self._items = items
        header = dump_yaml({key: field for key, field in value.items() if key != 'obstacles'})
        return header + 'obstacles:\n' + ''.join(texts)


class EnvelopeJson:
    """JSON text of envelopes and command results, encoding a document once.

    An envelope of a 2580-part scene is about 0.8 MB of JSON, but between
    publishes only its status, consumer and time fields change. Documents and
    their obstacles are immutable values shared between revisions, so the
    text of the current document is reused, and a new revision encodes only
    the obstacles it changed. The output equals json.dumps(value,
    ensure_ascii=False, allow_nan=False): a dict is written as that function
    writes it, '{' + ', '.join('"key": value') + '}', a list as
    '[' + ', '.join(items) + ']', every part encoded by the same function.
    """

    def __init__(self):
        # Each cache is replaced as one value: ROS callback threads share it.
        self._document = (None, None)
        self._obstacles = {}

    @staticmethod
    def _dumps(value):
        return json.dumps(value, ensure_ascii=False, allow_nan=False)

    def _object(self, value, encode):
        return '{' + ', '.join(self._dumps(key) + ': ' + encode(key, item) for key, item in value.items()) + '}'

    def document(self, document):
        cached = self._document
        if cached[0] is document:
            return cached[1]
        obstacles = document.get('obstacles')
        if not isinstance(obstacles, list) or not all(isinstance(key, str) for key in document):
            text = self._dumps(document)
        else:
            previous, items, texts = self._obstacles, {}, []
            for item in obstacles:
                hit = previous.get(id(item))
                if hit is None or hit[0] is not item:
                    hit = (item, self._dumps(item))
                items[id(item)] = hit
                texts.append(hit[1])
            self._obstacles = items
            listed = '[' + ', '.join(texts) + ']'
            text = self._object(document, lambda key, item: listed if key == 'obstacles' else self._dumps(item))
        self._document = (document, text)
        return text

    def dumps(self, value):
        if (not isinstance(value, dict) or not isinstance(value.get('document'), dict)
                or not all(isinstance(key, str) for key in value)):
            return self._dumps(value)
        return self._object(value, lambda key, item: self.document(item) if key == 'document' else self._dumps(item))


def yaml_object(loader, node):
    loader.flatten_mapping(node)
    return unique_object(loader.construct_pairs(node, deep=True))


SceneLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, yaml_object)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def load(path):
    with Path(path).open('rb') as stream:
        data = stream.read(MAX_DOCUMENT_BYTES+1)
    if len(data) > MAX_DOCUMENT_BYTES:
        raise SceneError('Scene file exceeds the supported size')
    try:
        return document(yaml.load(data, Loader=SceneLoader)), digest(data)
    except yaml.YAMLError as error:
        raise SceneError('Invalid scene YAML: {}'.format(error))


def file_digest(path):
    with Path(path).open('rb') as stream:
        data = stream.read(MAX_DOCUMENT_BYTES+1)
    if len(data) > MAX_DOCUMENT_BYTES:
        raise SceneError('Scene file exceeds the supported size')
    return digest(data)


FROZEN_REJECTED = frozenset(
    ('add', 'update', 'delete', 'clear', 'replace', 'undo', 'redo', 'save', 'reload'))
FROZEN_ERROR = 'Scene geometry is read-only for this simulator; select an editable scene to change obstacles'


class SceneStore:
    def __init__(self, initial, source=None, save_root=None, apply=None, clock=time.monotonic, source_digest=None,
                 frozen=False, working_file=None, motion=None):
        self.lock = threading.RLock()
        self.frozen = bool(frozen)
        self.working_file = Path(working_file).resolve() if working_file else None
        if self.working_file and self.working_file.exists():
            source = self.working_file
            initial, source_digest = load(source)
        self.document = document(initial)
        self.epoch = str(uuid.uuid4())
        self.revision = 1
        self.applied_revision = 0
        self.application_known = False
        self.next_revision = 2
        self.saved_revision = 1
        self.saved_document = self.document
        self.source = Path(source).resolve() if source else None
        save_target = self.working_file or self.source
        self.save_root = Path(save_root).resolve() if save_root else (save_target.parent if save_target else None)
        self.file_digests = {}
        if self.source and self.source.exists():
            self.file_digests[self.source] = source_digest or file_digest(self.source)
        self.apply = apply or (lambda doc, epoch, revision: None)
        self.motion = motion or (lambda operation, epoch, revision: None)
        self.clock = clock
        self.playing = False
        self.elapsed = 0.0
        self.started_at = clock()
        self.undo_stack = []
        self.redo_stack = []
        self.requests = OrderedDict()
        self.yaml = DocumentYaml()
        self.apply(self.document, self.epoch, self.revision)
        self.applied_revision = self.revision
        self.application_known = True

    def scene_time(self):
        return self.elapsed + (max(0.0, self.clock()-self.started_at) if self.playing else 0.0)

    def status(self):
        """Envelope fields other than the document and the running scene time."""
        with self.lock:
            return {'epoch': self.epoch, 'revision': self.revision, 'savedRevision': self.saved_revision,
                    'dirty': self.document is not self.saved_document and self.document != self.saved_document,
                    'playing': self.playing, 'frozen': self.frozen,
                    'configuration': {
                        'desired': {'epoch': self.epoch, 'revision': self.revision},
                        'applied': {'epoch': self.epoch, 'revision': self.applied_revision,
                                    'known': self.application_known},
                        'persisted': {'epoch': self.epoch, 'revision': self.saved_revision}}}

    def envelope(self):
        with self.lock:
            result = self.status()
            result.update(sceneTime=self.scene_time(), document=self.document)
            return result

    def states(self):
        with self.lock:
            elapsed = self.scene_time()
            return [state(item, elapsed, self.playing) for item in self.document['obstacles']]

    def command(self, request):
        with self.lock:
            try:
                fields(request, ('requestId', 'expectedEpoch', 'expectedRevision', 'operation', 'obstacle', 'id', 'document'), ('operation',))
                if request['operation'] == 'get':
                    return dict(self.envelope(), success=True)
                rid = request.get('requestId')
                if not isinstance(rid, str) or not re.fullmatch(r'[A-Za-z0-9._:-]{1,128}', rid, re.ASCII):
                    raise SceneError('Request ID must follow the XRPC identity contract')
                fingerprint = json.dumps(request, sort_keys=True, allow_nan=False)
                previous = self.requests.get(rid)
                if previous:
                    if previous[0] != fingerprint:
                        raise SceneError('Request ID was already used for a different operation')
                    return dict(previous[1])
                if request.get('expectedEpoch') != self.epoch or type(request.get('expectedRevision')) is not int or request['expectedRevision'] != self.revision:
                    raise SceneError('Scene changed; refresh before editing')
                try:
                    self._execute(request)
                    result = dict(self.envelope(), success=True)
                except (SceneError, OSError, ValueError, TypeError) as error:
                    result = dict(self.envelope(), success=False, error=str(error))
                self.requests[rid] = (fingerprint, result)
                while len(self.requests) > 256:
                    self.requests.popitem(last=False)
                return dict(result)
            except (SceneError, OSError, ValueError, TypeError) as error:
                return dict(self.envelope(), success=False, error=str(error))

    def _execute(self, request):
        operation = request['operation']
        if self.frozen and operation in FROZEN_REJECTED:
            raise SceneError(FROZEN_ERROR)
        if operation == 'resync':
            self.revision = self._apply(self.document)
            return
        if operation == 'save':
            self._check_source()
            self._save()
            return
        if operation == 'reload':
            if self.source is None:
                raise SceneError('This scene has no source YAML configured')
            replacement, source_digest = load(self.source)
            revision = self._apply(replacement)
            self.document = replacement
            self.revision = revision
            self.saved_document = replacement
            self.saved_revision = revision
            self.file_digests[self.source] = source_digest
            self.undo_stack.clear()
            self.redo_stack.clear()
            return
        if operation in ('play', 'pause', 'reset'):
            self.motion(operation, self.epoch, self.revision)
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
            self._prepare_edit()
            replacement = source[-1]
            revision = self._apply(replacement)
            target.append(self.document)
            source.pop()
            self.document = replacement
            self.revision = revision
            self._autosave()
            return
        # Only the edited obstacle is validated again; the others are already
        # normalized and shared with the current document.
        obstacles = list(self.document['obstacles'])
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
            candidate = dict(self.document, obstacles=check_obstacle_set(obstacles))
        elif operation == 'delete':
            oid = identifier(request.get('id'), 'Obstacle ID')
            remaining = [item for item in obstacles if item['id'] != oid]
            if len(remaining) == len(obstacles):
                raise SceneError('Obstacle no longer exists')
            candidate = dict(self.document, obstacles=remaining)
        elif operation == 'clear':
            candidate = dict(self.document, obstacles=[])
        elif operation == 'replace':
            candidate = document(request.get('document'))
        else:
            raise SceneError('Unsupported scene operation {!r}'.format(operation))
        if candidate == self.document:
            return
        self._prepare_edit()
        revision = self._apply(candidate)
        self.undo_stack.append(self.document)
        self.undo_stack = self.undo_stack[-128:]
        self.redo_stack.clear()
        self.document = candidate
        self.revision = revision
        self._autosave()

    def _prepare_edit(self):
        self._check_source()
        if self.working_file and self.source != self.working_file:
            # Establish the experiment's writable copy before changing live
            # geometry. Subsequent edits and reloads use only that copy.
            self._save()

    def _check_source(self):
        if self.source is not None:
            current = file_digest(self.source) if self.source.exists() else None
            if current != self.file_digests.get(self.source):
                raise SceneError('Scene YAML changed outside the editor; reload YAML before editing')

    def _autosave(self):
        if self.source is not None:
            try:
                self._save()
            except (SceneError, OSError) as error:
                raise SceneError('Live scene updated, but YAML was not saved: {}'.format(error)) from error

    def _apply(self, candidate):
        # Reserve the identity before transport. A lost reply may mean the
        # simulator applied this candidate, so a different candidate must never
        # reuse its revision. Resync can restore the accepted document at a new
        # revision after either a partial failure or an ambiguous lost reply.
        revision = self.next_revision
        self.next_revision += 1
        self.application_known = False
        self.apply(candidate, self.epoch, revision)
        self.applied_revision = revision
        self.application_known = True
        return revision

    def _save(self):
        if self.save_root is None:
            raise SceneError('This scene has no writable project directory configured')
        target = self.working_file or self.source
        if target is None:
            raise SceneError('This scene has no source YAML configured')
        try:
            target.relative_to(self.save_root)
        except ValueError:
            raise SceneError('Save target is outside the configured project directory')
        if target.suffix.lower() not in ('.yaml', '.yml'):
            raise SceneError('Save target must be a YAML file')
        if self.working_file:
            target.parent.mkdir(parents=True, exist_ok=True)
        if not target.parent.is_dir():
            raise SceneError('Save directory does not exist')
        current = file_digest(target) if target.exists() else None
        expected = self.file_digests.get(target)
        if current != expected:
            raise SceneError('Scene YAML changed outside the editor; reload YAML before editing')
        encoded = self.yaml.dump(self.document).encode('utf-8')
        handle, temporary = tempfile.mkstemp(prefix='.'+target.name+'.', suffix='.tmp', dir=str(target.parent))
        try:
            with os.fdopen(handle, 'wb') as stream:
                if target.exists():
                    os.fchmod(stream.fileno(), stat.S_IMODE(target.stat().st_mode))
                stream.write(encoded)
                stream.flush()
                os.fsync(stream.fileno())
            # Recheck after serialization; do not overwrite an external edit detected here.
            actual = file_digest(target) if target.exists() else None
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
        self.saved_document = self.document
        self.saved_revision = self.revision
