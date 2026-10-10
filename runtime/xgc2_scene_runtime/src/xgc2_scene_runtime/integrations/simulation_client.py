"""Explicitly bound simulation-v1 client; completion belongs to the world."""

import json
import time
import uuid

from xgc2_xrpc.http import Client, Fault, Limits, TransportError
from xgc2_xrpc.reference import ServiceRef

from ..document import SceneError


class SimulationClient:
    def __init__(self, reference, *, runtime, local_target, timeout=30.0):
        reference = ServiceRef.from_dict(reference)
        if reference.service != 'xgc2.simulation' or reference.api_version != 'v1':
            raise SceneError('An explicit simulation-v1 ServiceRef is required')
        reference.validate()
        self.reference = reference
        self.timeout = timeout
        self.client = Client.from_service(reference, runtime=runtime, local_target=local_target,
                                          limits=Limits(body_bytes=8*1024*1024,
                                                        response_bytes=8*1024*1024,
                                                        call_timeout=timeout))

    def operation(self, route, value, *, method='POST', request_id=None):
        """Submit once, then one notification-driven wait. Never replay writes."""
        request_id = request_id or uuid.uuid4().hex
        deadline = time.monotonic() + self.timeout
        payload = dict(value, operation_timeout_ms=max(1, int(self.timeout*1000)))
        try:
            result = self.client.json(route, payload, method=method, request_id=request_id,
                                      timeout=max(.001, deadline-time.monotonic()))
            if result.get('id') != request_id:
                raise SceneError('Simulation returned a different operation identity')
            if result.get('state') in ('accepted', 'running'):
                result = self.client.json('/v1/operations/{}/wait'.format(request_id), {},
                                          timeout=max(.001, deadline-time.monotonic()))
            if result.get('id') != request_id or result.get('state') != 'succeeded':
                raise SceneError('Simulation operation {} did not complete: {}'.format(
                    request_id, result.get('error') or result.get('state')))
            return result.get('result', {})
        except TransportError as error:
            raise SceneError('Simulation operation {}: {} ({})'.format(
                request_id, error, error.disposition)) from error
        except Fault as error:
            raise SceneError('Simulation operation {}: {} ({})'.format(
                request_id, error, error.code)) from error

    def apply(self, document, epoch, revision):
        result = self.operation('/v1/extensions/scene/apply',
                                {'document': document, 'epoch': epoch, 'revision': revision})
        if result.get('epoch') != epoch or result.get('revision') != revision:
            raise SceneError('Simulation did not commit the requested scene revision')
        return result

    def motion(self, operation, epoch, revision):
        return self.operation('/v1/extensions/scene/motion',
                              {'operation': operation, 'epoch': epoch, 'revision': revision})

    def close(self):
        self.client.close()
