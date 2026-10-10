#!/usr/bin/env python3
"""Run unchanged point-cloud lifecycle assertions on a private ROS master."""
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest

def main():
    if not os.environ.get('XGC2_WORLD_LIDAR_FIXTURE'):
        raise ValueError('XGC2_WORLD_LIDAR_FIXTURE must name the explicitly built node')
    os.environ.pop('DISPLAY', None)
    os.environ.pop('WAYLAND_DISPLAY', None)
    with tempfile.TemporaryDirectory(prefix='xgc2-lidar-data-') as directory:
        with socket.socket() as reserved:
            reserved.bind(('127.0.0.1', 0)); port = reserved.getsockname()[1]
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:'+str(port)
        os.environ['ROS_IP'] = '127.0.0.1'
        os.environ['ROS_HOME'] = directory
        os.environ['ROS_LOG_DIR'] = str(Path(directory)/'logs')
        os.environ.pop('ROS_HOSTNAME', None)
        import rosgraph
        import rospy
        sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
        from cache_lifecycle_test import CacheLifecycle
        with (Path(directory)/'master.log').open('w') as log:
            master = subprocess.Popen(['rosmaster', '--core', '-p', str(port)], stdout=log, stderr=log)
            try:
                deadline = time.monotonic()+10
                while True:
                    if master.poll() is not None:
                        log.flush(); raise RuntimeError('private fixture master exited: '+(Path(directory)/'master.log').read_text())
                    try:
                        rosgraph.Master('/lidar_data_fixture').getPid(); break
                    except (OSError, rosgraph.MasterException):
                        if time.monotonic() >= deadline: raise RuntimeError('private fixture master did not start')
                        time.sleep(.02)
                rospy.init_node('lidar_data_fixture', disable_signals=True)
                result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(CacheLifecycle))
                if not result.wasSuccessful():
                    log.flush(); raise AssertionError('native data callback regression failed; private master log: '+(Path(directory)/'master.log').read_text())
            finally:
                rospy.signal_shutdown('isolated fixture ended')
                master.terminate()
                try: master.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    master.kill(); master.wait()

if __name__ == '__main__': main()
