"""Real SDK/UDS controls: instance fences, receipts and completed world effects."""
import json
from pathlib import Path
import tempfile
import unittest

from xgc2_xrpc.http import Client, Fault, Host, TransportError
from xgc2_xrpc.runtime import Runtime
from xgc2_scene_runtime.document import SceneError
from xgc2_scene_runtime.simulation_client import SimulationClient
from xgc2_scene_runtime.store import SceneStore
from xgc2_scene_runtime.xrpc_service import SceneService

DOCUMENT = {'schema': 'xgc2.scene.v1', 'id': 'test', 'frame': 'world', 'obstacles': []}


class Node:
    online = True
    def __init__(self):
        self.store = SceneStore(DOCUMENT)
    def envelope(self):
        return self.store.envelope()
    def command_value(self, value):
        return self.store.command(value)


class ControlTest(unittest.TestCase):
    def test_real_authoring_host_and_retained_receipts(self):
        with tempfile.TemporaryDirectory() as root:
            runtime = Runtime(blocking_workers=2)
            node = Node()
            service = SceneService(node, path=str(Path(root)/'scene.sock'), runtime=runtime, target_id='fixture')
            reference = service.start()
            client = Client(reference['endpoint']['address'], runtime=runtime, instance_id=reference['instance_id'])
            try:
                before = client.json('/v1/scene', method='GET')
                command = {'operation': 'play', 'expectedEpoch': before['epoch'], 'expectedRevision': before['revision']}
                result = client.json('/v1/commands', command, request_id='native:play')
                self.assertTrue(result['success'])
                self.assertTrue(result['playing'])
                receipt = client.json('/v1/commands/result', {'request_id': 'native:play'})
                self.assertEqual(receipt, result)
                self.assertEqual(result, client.json('/v1/commands', command, request_id='native:play'))
                conflict = client.json('/v1/commands', dict(command, operation='pause'), request_id='native:play')
                self.assertFalse(conflict['success'])
                with self.assertRaises(Fault):
                    client.json('/v1/commands', dict(command, requestId='payload-id'))
                client.instance_id = 'stale'
                with self.assertRaises(TransportError) as raised:
                    client.json('/v1/scene', method='GET')
                self.assertEqual(raised.exception.disposition, 'outcome_unknown')
            finally:
                client.close(); service.close(); runtime.close()
            self.assertFalse(Path(reference['endpoint']['address']).exists())

    def test_simulation_waits_once_and_never_probes_entities(self):
        with tempfile.TemporaryDirectory() as root:
            runtime = Runtime(blocking_workers=2)
            calls = []
            def submit(context, value):
                calls.append(('submit', context.request_id))
                return {'id': context.request_id, 'state': 'accepted'}
            def wait(context, value):
                calls.append(('wait', context.request_id))
                return {'id': 'create-one', 'state': 'succeeded', 'result': {'entity': {'id': 'one', 'generation': 1}}}
            path = str(Path(root)/'world.sock')
            host = Host(path, {('POST', '/v1/entities'): submit,
                               ('POST', '/v1/operations/create-one/wait'): wait}, runtime=runtime,
                        instance_id='world-instance')
            host.start()
            reference = {'target_id': 'fixture', 'service': 'xgc2.simulation', 'api_version': 'v1',
                         'instance_id': 'world-instance', 'profile': 'http.v1',
                         'endpoint': {'kind': 'unix', 'address': path}}
            client = SimulationClient(reference, runtime=runtime, local_target='fixture')
            try:
                result = client.operation('/v1/entities', {'entity': {}}, request_id='create-one')
                self.assertEqual(result['entity']['generation'], 1)
                self.assertEqual([kind for kind, _ in calls], ['submit', 'wait'])
                self.assertNotEqual(calls[0][1], calls[1][1])
            finally:
                client.close(); host.close(); runtime.close()

    def test_failed_native_effect_does_not_promote_desired_or_persisted(self):
        def apply(document, epoch, revision):
            if revision > 1:
                raise SceneError('native outcome unknown')
        store = SceneStore(DOCUMENT, apply=apply)
        candidate = dict(DOCUMENT, id='replacement')
        result = store.command({'operation': 'replace', 'document': candidate, 'requestId': 'lost-write',
                                'expectedEpoch': store.epoch, 'expectedRevision': 1})
        self.assertFalse(result['success'])
        self.assertEqual(store.revision, 1)
        self.assertFalse(result['configuration']['applied']['known'])
        self.assertEqual(result['configuration']['persisted']['revision'], 1)


if __name__ == '__main__':
    unittest.main()
