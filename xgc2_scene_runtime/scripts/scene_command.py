#!/usr/bin/env python3
"""Send one edit to the running scene author's public service."""
import argparse
import json
import sys
from xgc2_xrpc.http import Client, Limits
from xgc2_xrpc.reference import ServiceRef
from xgc2_xrpc.runtime import Runtime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--service-ref-json', required=True)
    parser.add_argument('--command-json', required=True)
    args = parser.parse_args()
    reference = ServiceRef.from_dict(json.loads(args.service_ref_json)).validate()
    if reference.service != 'xgc2.scene.authoring' or reference.api_version != 'v1':
        raise ValueError('The actual scene authoring ServiceRef is required')
    if len(args.command_json.encode()) > 65536:
        raise ValueError('Scene command exceeds the CLI byte limit')
    command = json.loads(args.command_json)
    request_id = command.pop('requestId', None)
    runtime = Runtime(blocking_workers=1, max_connections=1)
    client = Client.from_service(reference, runtime=runtime, local_target=reference.target_id,
                                 limits=Limits(call_timeout=60, body_bytes=65536, response_bytes=65536, connections=1))
    try:
        if command == {'operation':'get'}:
            result = dict(client.json('/v1/scene', method='GET'), success=True)
        else:
            if not request_id:
                raise ValueError('A scene mutation requires its authored request identity')
            result = client.json('/v1/commands', command, request_id=request_id)
        print(json.dumps({'resultJson':json.dumps(result, ensure_ascii=False, allow_nan=False)},
                         ensure_ascii=False, allow_nan=False, separators=(',', ':')))
    finally:
        client.close()
        runtime.close()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print('scene_command: '+str(error), file=sys.stderr)
        sys.exit(1)
