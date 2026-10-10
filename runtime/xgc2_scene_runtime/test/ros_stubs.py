"""Minimal ROS stand-ins so SceneNode runs without a ROS installation.

Messages are plain attribute bags; publishers, broadcasters and the Gazebo
apply service record what the node sends so tests can count calls and bytes.
"""

import sys
import types


class Msg:
    _lists = ()

    def __init__(self, **values):
        for name in self._lists:
            object.__setattr__(self, name, [])
        for name, value in values.items():
            setattr(self, name, value)

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        child = Msg()
        object.__setattr__(self, name, child)
        return child


class Point(Msg):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        super().__init__(x=x, y=y, z=z)


class String(Msg):
    def __init__(self, data=''):
        super().__init__(data=data)


class Time:
    current = 1000.0

    def __init__(self, secs=0, nsecs=0):
        self.secs = secs + nsecs * 1e-9

    @classmethod
    def now(cls):
        return cls(cls.current)

    def to_sec(self):
        return self.secs

    def __bool__(self):
        return self.secs != 0


class Duration:
    def __init__(self, secs=0.0):
        self.secs = secs


class Marker(Msg):
    _lists = ('points',)
    ADD, DELETEALL = 0, 3
    CUBE, SPHERE, CYLINDER, TRIANGLE_LIST = 1, 2, 3, 11


class MarkerArray(Msg):
    _lists = ('markers',)


class SceneSnapshot(Msg):
    _lists = ('obstacles',)


class SceneObstacle(Msg):
    _lists = ('parts',)


class SceneState(Msg):
    _lists = ('obstacles',)


class SceneCommandResponse(Msg):
    pass


class ServiceException(Exception):
    pass


class ROSException(Exception):
    pass


class Recorder:
    """Everything the node sent, for one test."""

    def __init__(self):
        self.publishes = {}
        self.published_bytes = {}
        self.last = {}
        self.transforms = {'static': 0, 'dynamic': 0}
        self.services = {}
        self.subscribers = {}
        self.params = {}
        self.proxies = {}

    def publish(self, topic, message):
        self.publishes[topic] = self.publishes.get(topic, 0) + 1
        self.last[topic] = message
        if isinstance(message, String):
            self.published_bytes[topic] = self.published_bytes.get(topic, 0) + len(message.data.encode('utf-8'))


recorder = Recorder()


class Publisher:
    def __init__(self, topic, _type, queue_size=None, latch=False):
        self.topic = topic

    def publish(self, message):
        recorder.publish(self.topic, message)


class Subscriber:
    def __init__(self, topic, _type, callback, queue_size=None):
        recorder.subscribers[topic] = callback


class Service:
    def __init__(self, name, _type, handler):
        recorder.services[name] = handler


def ServiceProxy(name, _type):
    return recorder.proxies[name]


class Timer:
    def __init__(self, _period, callback, reset=False):
        self.callback = callback


class Broadcaster:
    kind = 'dynamic'

    def sendTransform(self, transforms):
        recorder.transforms[self.kind] += len(transforms) if isinstance(transforms, list) else 1


class StaticBroadcaster(Broadcaster):
    kind = 'static'


def install():
    """Register the stand-ins as rospy, tf2_ros and the message packages."""
    def module(name, **attributes):
        value = types.ModuleType(name)
        value.__dict__.update(attributes)
        sys.modules[name] = value
        return value

    module('rospy', Publisher=Publisher, Subscriber=Subscriber, Service=Service, ServiceProxy=ServiceProxy,
           Timer=Timer, Time=Time, Duration=Duration, ServiceException=ServiceException, ROSException=ROSException,
           get_param=lambda name, default=None: recorder.params.get(name, default),
           get_namespace=lambda: '/xgc/scene/', wait_for_service=lambda name, timeout=None: None,
           on_shutdown=lambda hook: None, init_node=lambda *args, **kwargs: None, spin=lambda: None,
           logfatal=lambda *args: None, loginfo=lambda *args: None)
    module('tf2_ros', StaticTransformBroadcaster=StaticBroadcaster, TransformBroadcaster=Broadcaster)
    module('geometry_msgs')
    module('geometry_msgs.msg', Point=Point, TransformStamped=Msg, Pose=Msg)
    module('std_msgs')
    module('std_msgs.msg', String=String)
    module('visualization_msgs')
    module('visualization_msgs.msg', Marker=Marker, MarkerArray=MarkerArray)
    module('xgc2_geometry_msgs')
    module('xgc2_geometry_msgs.msg', SceneSnapshot=SceneSnapshot, SceneObstacle=SceneObstacle, ScenePart=Msg,
           SceneState=SceneState, SceneObstacleState=Msg, SceneConsumerStatus=Msg)
    module('xgc2_geometry_msgs.srv', ApplyScene=Msg, SceneCommand=Msg, SceneCommandResponse=SceneCommandResponse)
    return recorder


class SceneService:
    """Projection tests do not run a transport. Real UDS is tested separately."""
    def __init__(self, node, **kwargs):
        self.node = node
    def start(self):
        return {'service': 'xgc2.scene.authoring', 'instance_id': 'projection-fixture'}
    def close(self):
        pass
