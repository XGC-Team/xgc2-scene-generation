#!/usr/bin/env python3
"""Prepare this product's sensor fleet from opaque frozen Experiment facts."""
import json
import math
import os
from pathlib import Path
import re
import sys


def prepare(envelope):
    context, robots = envelope['context'], envelope['robots']
    if len(robots) > 512:
        raise ValueError('world lidar supports at most 512 robots')
    fleet = dict(schemaVersion=1, robots=[])
    if not context['scene'] or context['runMode'] == 'physical' or (context['scene'] or {}).get('simulator') == 'xsim':
        return fleet
    for robot in robots:
        if context['runMode'] == 'hybrid' and robot['hybridSource'] != 'simulation':
            continue
        authored = robot['authoredSimulationSensors']['simpleLidar']
        lidar = dict(enabled=authored) if isinstance(authored, bool) else authored
        mode, preset = lidar.get('mode', ''), lidar.get('preset', '')
        if not lidar.get('enabled', False) or not (mode in ('penetrating', 'depth_frustum') or lidar.get('publishBeams', False) or (preset and mode != 'raycast')):
            continue
        if lidar.get('acceleration', '') != 'cpu':
            raise ValueError('sampled-scene lidar requires an explicit CPU acceleration selection')
        namespace = '/' + robot['namespace'].lstrip('/')
        if len(namespace) > 160 or not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', namespace):
            raise ValueError('invalid lidar namespace')
        sensor = dict(namespace=namespace, mode='raycast', rateHz=10, rangeMeters=20,
                      hFovDeg=360, vFovDeg=30, hRes=360, vRes=32, surfaceSpacing=.1,
                      keepBuried=False, headingCrop=False, headingCosMin=0, verticalSlabTan=0,
                      publishBeams=lidar.get('publishBeams', False))
        if preset:
            sensor.update(mode='penetrating', vFovDeg=180, keepBuried=True, rangeMeters=5)
            if preset == 'bridge_equivalent':
                sensor['rangeMeters'] = 8
            else:
                sensor.update(headingCrop=True, verticalSlabTan=1/math.sqrt(3))
                if preset == 'zju_cpu_crop': sensor['headingCosMin'] = .5
        if mode: sensor['mode'] = mode
        if sensor['mode'] == 'depth_frustum': sensor.update(hFovDeg=90, vFovDeg=60)
        for key in ('rateHz', 'rangeMeters', 'hFovDeg', 'vFovDeg', 'hRes', 'vRes'):
            if lidar.get(key, 0) > 0: sensor[key] = lidar[key]
        if sensor['mode'] == 'penetrating' and sensor['publishBeams']:
            raise ValueError('penetrating sampled clouds have no beams')
        fleet['robots'].append(sensor)
    for layer in context['visualizationTopics']:
        for topic in layer:
            if (topic['topic'] == '/xgc/scene/reference_cloud' and topic['messageType'] == 'sensor_msgs/PointCloud2'
                    and topic['role'] == 'semantic' and (topic.get('visible3d') or topic.get('visibleAr'))):
                fleet['referenceCloud'] = dict(surfaceSpacing=.1)
    return fleet


def main():
    raw = sys.stdin.buffer.read(8388609)
    if len(raw) > 8388608: raise ValueError('frozen Experiment input exceeds 8 MiB')
    envelope = json.loads(raw)
    fleet = prepare(envelope)
    if not fleet['robots'] and 'referenceCloud' not in fleet:
        print('{"required":false,"manifestPath":""}')
        return
    root = Path(os.environ['XGC_JOB_ARTIFACT_ROOT'])
    if not root.is_absolute(): raise ValueError('artifact root must be absolute')
    owner = envelope['context']['openingRunId']
    if not re.fullmatch(r'[A-Za-z0-9-]{1,128}', owner): raise ValueError('invalid Run artifact identity')
    directory = root/'world-lidar'/owner
    directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    output = directory/'fleet.json'
    payload = json.dumps(fleet, allow_nan=False, separators=(',', ':')).encode('utf-8')
    if len(payload) > 1048576: raise ValueError('fleet manifest exceeds 1 MiB')
    with open(output, 'wb') as stream:
        os.chmod(output, 0o600)
        stream.write(payload)
    print(json.dumps(dict(required=True, manifestPath=str(output)), separators=(',', ':')))


if __name__ == '__main__': main()
