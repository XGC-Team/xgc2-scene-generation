#!/usr/bin/env python3
"""Read readiness from the native simulation owner, without ROS control probes."""
import argparse
import json
import sys
import time
from xgc2_xrpc.http import Client, Limits
from xgc2_xrpc.reference import ServiceRef
from xgc2_xrpc.runtime import Runtime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', required=True)
    parser.add_argument('--target-id', required=True)
    parser.add_argument('--service', default='xgc2.simulation')
    parser.add_argument('--field', required=True, choices=('lifecycle', 'state'))
    parser.add_argument('--timeout', type=float, default=1)
    args = parser.parse_args()
    if not 0 < args.timeout <= 180:
        parser.error('timeout must be in (0, 180]')
    runtime = Runtime(blocking_workers=1, max_connections=1)
    client = None
    try:
        deadline = time.monotonic()+args.timeout
        discovery = ServiceRef.from_dict(dict(target_id=args.target_id, service=args.service,
                                              api_version='v1', profile='http.v1', instance_id='',
                                              endpoint={'kind':'unix', 'address':args.socket}))
        limits = Limits(call_timeout=args.timeout, response_bytes=65536, connections=1)
        client = Client.from_service(discovery, runtime=runtime, local_target=args.target_id, discovery=True, limits=limits)
        described = client.json('/v1/describe', method='GET', timeout=max(.001, deadline-time.monotonic()))
        actual = ServiceRef.from_dict(described['service_ref']).validate()
        if (actual.target_id, actual.service, actual.api_version, actual.profile, actual.endpoint) != (discovery.target_id, discovery.service, discovery.api_version, discovery.profile, discovery.endpoint):
            raise ValueError('The described native service does not match its declared endpoint')
        client.close()
        client = Client.from_service(actual, runtime=runtime, local_target=args.target_id, limits=limits)
        health = client.json('/v1/health', method='GET', timeout=max(.001, deadline-time.monotonic()))
        while health[args.field] != 'ready':
            if health[args.field] in ('failed', 'stopping', 'stopped'):
                raise ValueError('Native simulation is '+health[args.field])
            if args.timeout <= 1:
                raise ValueError('Native simulation is not ready')
            health = client.json('/v1/health/observe', {'after_revision':health['revision']},
                                 timeout=max(.001, deadline-time.monotonic()))
        print(json.dumps({'service_ref':dict(actual.__dict__, endpoint=actual.endpoint.__dict__), 'health':health}, separators=(',', ':')))
        return 0
    except (Exception,) as error:
        print('simulation_ready: '+str(error), file=sys.stderr)
        return 1
    finally:
        if client is not None: client.close()
        runtime.close()


if __name__ == '__main__': sys.exit(main())
