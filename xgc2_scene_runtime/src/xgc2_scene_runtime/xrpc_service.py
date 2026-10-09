"""The authoring process owns edits/persistence, the simulation owns effects."""

import uuid

from xgc2_xrpc.http import Fault, Host, Limits

from .document import SceneError
from .store import MAX_DOCUMENT_BYTES


class SceneService:
    def __init__(self, node, *, path, runtime, target_id):
        self.node = node
        self.reference = {'target_id': target_id, 'service': 'xgc2.scene.authoring',
                          'api_version': 'v1', 'instance_id': uuid.uuid4().hex,
                          'profile': 'http.v1', 'endpoint': {'kind': 'unix', 'address': path}}
        self.host = Host(path, {('GET', '/v1/describe'): self.describe,
                                ('GET', '/v1/health'): self.health,
                                ('GET', '/v1/scene'): self.scene,
                                ('POST', '/v1/commands'): self.command,
                                ('POST', '/v1/commands/result'): self.result}, runtime=runtime,
                         limits=Limits(in_flight=4, body_bytes=MAX_DOCUMENT_BYTES,
                                       response_bytes=MAX_DOCUMENT_BYTES+65536, call_timeout=60.0),
                         instance_id=self.reference['instance_id'], discovery_routes=('/v1/describe',))

    def start(self):
        self.host.start()
        return self.reference

    def describe(self, _context, _value):
        return {'service_ref': self.reference, 'capabilities': ['scene.read', 'scene.edit',
                 'scene.motion', 'scene.save'], 'limits': {'document_bytes': MAX_DOCUMENT_BYTES,
                  'in_flight': 4, 'request_results': 256},
                'completion': 'The command returns after the world operation completes. '
                              'Transport loss can leave the result unknown; writes are not replayed.'}

    def health(self, _context, _value):
        return {'lifecycle': 'ready' if self.node.online else 'stopping'}

    def scene(self, _context, _value):
        return self.node.envelope()

    def command(self, context, value):
        if not isinstance(value, dict) or 'requestId' in value:
            raise Fault('invalid_argument', 'Command identity must come from X-Request-ID')
        context.check_cancelled()
        result = self.node.command_value(dict(value, requestId=context.request_id))
        return result

    def result(self, _context, value):
        if not isinstance(value, dict) or set(value) != {'request_id'} or not isinstance(value['request_id'], str):
            raise Fault('invalid_argument', 'request_id is required')
        with self.node.store.lock:
            entry = self.node.store.requests.get(value['request_id'])
            if entry is None:
                raise Fault('not_found', 'Request result is unknown or outside retained history')
            return dict(entry[1])

    def close(self):
        self.host.close()
