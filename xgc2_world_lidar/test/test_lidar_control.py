"""Real native source owner, ROS callback dispatcher, and shared Python SDK."""
import asyncio
import os
from pathlib import Path
import tempfile
import unittest

from xgc2_xrpc.http import Client, Fault, TransportError
from xgc2_xrpc.runtime import Runtime


class LidarControlTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = str(Path(self.directory.name)/'source.sock')
        executable = os.environ['XGC2_LIDAR_CONTROL_FIXTURE']
        self.process = await asyncio.create_subprocess_exec(executable, self.path,
            stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        self.assertEqual(await asyncio.wait_for(self.process.stdout.readline(), 5), b'ready\n')
        self.runtime = Runtime(blocking_workers=2)
        self.discovery = Client(self.path, runtime=self.runtime)
        described = await self.json(self.discovery, '/v1/describe', method='GET')
        self.reference = described['service_ref']
        self.discovery.instance_id = self.reference['instance_id']
        self.client = Client(self.path, runtime=self.runtime, instance_id=self.reference['instance_id'])

    async def asyncTearDown(self):
        self.client.close(); self.discovery.close(); self.runtime.close()
        if self.process.returncode is None:
            self.process.stdin.write(b'stop\n'); await self.process.stdin.drain()
            await asyncio.wait_for(self.process.wait(), 5)
        self.assertEqual(self.process.returncode, 0, (await self.process.stderr.read()).decode())
        self.assertFalse(Path(self.path).exists())
        self.directory.cleanup()

    async def json(self, client, *args, **kwargs):
        return await asyncio.to_thread(client.json, *args, **kwargs)

    async def dispatch(self):
        self.process.stdin.write(b'dispatch\n'); await self.process.stdin.drain()
        return (await asyncio.wait_for(self.process.stdout.readline(), 5)).decode().strip()

    async def admitted(self, revision):
        # The fixture deliberately suspends the source queue. Observe one
        # admitted status explicitly; this is not provider/client retry logic.
        for _ in range(100):
            state = await self.json(self.discovery, '/v1/status', method='GET')
            if state['desired']['revision'] == revision: return state
            await asyncio.sleep(.005)
        self.fail('configure was not admitted')

    async def test_completed_effect_cas_recovery_and_instance_fence(self):
        body = {'expected_revision': 1, 'configuration': {'enabled': False}, 'operation_timeout_ms': 1000}
        task = asyncio.create_task(self.json(self.client, '/v1/configure', body, request_id='disable-once'))
        state = await self.admitted(2)
        self.assertTrue(state['applied']['configuration']['enabled'])
        self.assertFalse(task.done())
        self.assertEqual(await self.dispatch(), 'false 1')
        result = await task
        self.assertEqual(result['state'], 'succeeded')
        self.assertFalse(result['result']['configuration']['enabled'])
        self.assertEqual(await self.json(self.client, '/v1/configure', body, request_id='disable-once'), result)
        self.assertEqual(await self.dispatch(), 'false 1')
        self.assertEqual(await self.json(self.client, '/v1/operations/disable-once', method='GET'), result)
        with self.assertRaises(Fault):
            await self.json(self.client, '/v1/configure', body, request_id='stale-cas')
        for bad in [dict(body, expected_revision=2.0), dict(body, expected_revision=2, operation_timeout_ms=1.5),
                    dict(body, expected_revision=2, configuration={'enabled': 1}), dict(body, expected_revision=2, typo=1)]:
            with self.subTest(bad=bad), self.assertRaises(Fault):
                await self.json(self.client, '/v1/configure', bad)
        self.client.instance_id = 'stale-source'
        with self.assertRaises((Fault, TransportError)):
            await self.json(self.client, '/v1/status', method='GET')

    async def test_expired_operation_never_calls_source(self):
        body = {'expected_revision': 1, 'configuration': {'enabled': False}, 'operation_timeout_ms': 1}
        task = asyncio.create_task(self.json(self.client, '/v1/configure', body, request_id='expire-before-dispatch'))
        await self.admitted(2); await asyncio.sleep(.02)
        self.assertEqual(await self.dispatch(), 'true 0')
        result = await task
        self.assertEqual(result['state'], 'failed')
        self.assertEqual(result['error']['code'], 'deadline_exceeded')
        state = await self.json(self.client, '/v1/status', method='GET')
        self.assertEqual(state['applied']['revision'], 1)
        self.assertIsNone(state['persisted'])


if __name__ == '__main__':
    unittest.main()
