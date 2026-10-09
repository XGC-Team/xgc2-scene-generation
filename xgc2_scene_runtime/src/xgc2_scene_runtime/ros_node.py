"""ROS user-data projections and the XRPC authoring host."""

import hashlib
import json
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, TransformStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from xgc2_geometry_msgs.msg import (
    SceneSnapshot, SceneObstacle, ScenePart, SceneState, SceneObstacleState,
)
from xgc2_xrpc.runtime import Runtime

from .document import SceneError
from .generation import resolve
from .motion import rotate, state
from .store import MAX_DOCUMENT_BYTES, EnvelopeJson, SceneStore, load, unique_object
from .simulation_client import SimulationClient
from .xrpc_service import SceneService


def set_pose(message, value):
    message.position.x, message.position.y, message.position.z = value['position']
    message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = value['orientation']


def obstacle_message(item):
    body = SceneObstacle()
    body.id, body.name = item['id'], item['name']
    body.motion_type = item['motion']['type']
    body.dynamic = body.motion_type != 'hold'
    set_pose(body.pose, item['pose'])
    for definition in item['parts']:
        part = ScenePart()
        part.id = definition['id']
        set_pose(part.pose, definition['pose'])
        geometry = definition['geometry']
        part.geometry.type = geometry['type']
        part.geometry.size.x, part.geometry.size.y, part.geometry.size.z = geometry.get('size', [0, 0, 0])
        part.geometry.radius = geometry.get('radius', 0)
        part.geometry.height = geometry.get('height', 0)
        part.geometry.vertices = [Point(*p) for p in geometry.get('vertices', [])]
        part.geometry.triangles = geometry.get('triangles', [])
        part.color.r, part.color.g, part.color.b, part.color.a = definition['color']
        body.parts.append(part)
    return body


class ObstacleMessages:
    """SceneObstacle messages reused while their obstacle object is unchanged.

    Documents are immutable and share unchanged obstacles between revisions,
    so an edit builds the message of the edited obstacle only. Messages are
    never modified after they are built.
    """

    def __init__(self):
        self._items = {}

    def bodies(self, obstacles):
        items, bodies = {}, []
        for item in obstacles:
            cached = self._items.get(id(item))
            if cached is None or cached[0] is not item:
                cached = (item, obstacle_message(item))
            items[id(item)] = cached
            bodies.append(cached[1])
        self._items = items
        return bodies


def snapshot(document, epoch, revision, messages=None):
    result = SceneSnapshot()
    result.header.stamp = rospy.Time.now()
    result.header.frame_id = document['frame']
    result.scene_id, result.epoch, result.revision = document['id'], epoch, revision
    if messages is None:
        result.obstacles.extend(obstacle_message(item) for item in document['obstacles'])
    else:
        result.obstacles.extend(messages.bodies(document['obstacles']))
    return result


def state_message(value):
    body = SceneObstacleState()
    body.id = value['id']
    set_pose(body.pose, value['pose'])
    body.twist.linear.x, body.twist.linear.y, body.twist.linear.z = value['linear']
    body.twist.angular.x, body.twist.angular.y, body.twist.angular.z = value['angular']
    return body


class SceneNode:
    def __init__(self):
        source = rospy.get_param('~scene_file')
        overrides = json.loads(rospy.get_param('~generation_parameters_json', '{}'))
        (initial, source_digest), source = resolve(source, rospy.get_param('~working_file', '') or '', overrides)
        self.gazebo = bool(rospy.get_param('~gazebo', True))
        self.frozen = bool(rospy.get_param('~frozen', False))
        self.native_report = None
        self.consumer_lock = threading.RLock()
        self.online = True
        self.snapshot_pub = rospy.Publisher('snapshot', SceneSnapshot, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher('state', SceneState, queue_size=1)
        self.document_pub = rospy.Publisher('document', String, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('markers', MarkerArray, queue_size=1, latch=True)
        self.tf_static = tf2_ros.StaticTransformBroadcaster()
        self.tf_dynamic = tf2_ros.TransformBroadcaster()
        self.frame_prefix = 'xgc_scene_' + hashlib.sha256(rospy.get_namespace().encode()).hexdigest()[:12]
        # Per-revision caches: obstacle TF frames, and the snapshot built for
        # the Gazebo apply that the definition publish would otherwise rebuild.
        self._frames_key, self._frames = None, {}
        self._applied_snapshot = (None, None)
        # Consumer heartbeats republish the document only when its public
        # view changed; it is latched, so repeats carry no information.
        self._published_view = None
        # Per document: obstacle messages and markers of unchanged obstacles,
        # the encoded document, and the constant state of hold obstacles.
        self._obstacle_messages = ObstacleMessages()
        self._markers = {}
        self._json = EnvelopeJson()
        self._states = (None, [])
        self.xrpc_runtime = Runtime(blocking_workers=2, max_calls=8)
        self.simulation = None
        if self.gazebo:
            self.simulation = SimulationClient(json.loads(rospy.get_param('~simulation_service_ref_json')),
                                                runtime=self.xrpc_runtime,
                                                local_target=rospy.get_param('~target_id'))
        self.store = SceneStore(initial, source, rospy.get_param('~save_directory', '') or None,
                                self.apply_scene, lambda: rospy.Time.now().to_sec(), source_digest=source_digest,
                                frozen=self.frozen, working_file=rospy.get_param('~working_file', '') or None,
                                motion=self.apply_motion)
        self.service = SceneService(self, path=rospy.get_param('~xrpc_socket'), runtime=self.xrpc_runtime,
                                    target_id=rospy.get_param('~target_id'))
        self.service_ref = self.service.start()
        self.publish_definition()
        self.timer = rospy.Timer(rospy.Duration(1.0/30.0), self.tick, reset=True)
        rospy.on_shutdown(self.shutdown)

    def apply_scene(self, document, epoch, revision):
        if self.simulation is None:
            return
        try:
            message = snapshot(document, epoch, revision, self._obstacle_messages)
            self._applied_snapshot = ((epoch, revision), message)
            result = self.simulation.apply(document, epoch, revision)
        except SceneError as error:
            self.gazebo_failure(epoch, revision, str(error))
            raise
        self._remember({
            'consumer': 'gazebo', 'epoch': epoch, 'revision': revision,
            'applied': True, 'operational': True, 'capability': 'ok',
            'message': 'Native scene operation completed', 'header_stamp': rospy.Time.now().to_sec(),
        })

    def apply_motion(self, operation, epoch, revision):
        if self.simulation is not None:
            self.simulation.motion(operation, epoch, revision)

    def gazebo_failure(self, epoch, revision, message):
        # A failed factory operation can leave a partially changed simulator.
        # The accepted document stays intact, but the old success must not mask this.
        self._remember({
            'consumer': 'gazebo', 'epoch': epoch, 'revision': revision,
            'applied': False, 'operational': False, 'capability': '',
            'message': message, 'header_stamp': rospy.Time.now().to_sec(),
        })

    def _remember(self, report):
        with self.consumer_lock:
            self.native_report = report

    def application_view(self, epoch, revision):
        with self.consumer_lock:
            rows = [] if self.native_report is None else [self.native_report]
            synchronized = all(row['applied'] and row['epoch'] == epoch and row['revision'] == revision for row in rows)
        return {'consumers': rows, 'synchronized': synchronized, 'syncRetryable': not synchronized}

    def envelope(self):
        result = self.store.envelope()
        public = self.application_view(result['epoch'], result['revision'])
        result['consumers'] = public['consumers']
        result['online'] = self.online
        result['synchronized'] = public['synchronized']
        result['syncRetryable'] = public['syncRetryable']
        return result

    def publish_document(self, changed_only=False):
        """Publish the envelope; with changed_only, skip a view already published.

        The view is everything in the envelope except the running scene time,
        which subscribers take from the state topic.
        """
        with self.store.lock:
            status = self.store.status()
            public = self.application_view(status['epoch'], status['revision'])
            view = json.dumps([status, public, self.online], sort_keys=True)
            if changed_only and view == self._published_view:
                return False
            result = dict(status, sceneTime=self.store.scene_time(), document=self.store.document,
                          consumers=public['consumers'], online=self.online, synchronized=public['synchronized'],
                          syncRetryable=public['syncRetryable'])
            self.document_pub.publish(String(self._json.dumps(result)))
            self._published_view = view
            return True

    def command_value(self, command):
        try:
            with self.store.lock:
                previous = self.store.revision
                result = self.store.command(command)
                if self.store.revision != previous:
                    self.publish_definition()
                elif command.get('operation') != 'get':
                    self.tick(None)
                    self.publish_document()
                result.update(self.envelope())
        except (ValueError, TypeError, AttributeError) as error:
            result = dict(self.envelope(), success=False, error=str(error))
        return result

    def obstacle_frames(self):
        """TF child frame per obstacle ID, rebuilt once per document revision."""
        key = (self.store.epoch, self.store.revision)
        if key != self._frames_key:
            # tf2 caches static transforms indefinitely. Switching motion modes must
            # not reuse the same child frame for both static and dynamic TF data.
            self._frames = {
                item['id']: '{}/{}/{}/{}'.format(self.frame_prefix, self.store.epoch,
                                                 'fixed' if item['motion']['type'] == 'hold' else 'moving', item['id'])
                for item in self.store.document['obstacles']}
            self._frames_key = key
        return self._frames

    def obstacle_frame(self, oid):
        return self.obstacle_frames()[oid]

    def transform(self, oid, pose, stamp):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.store.document['frame']
        transform.child_frame_id = self.obstacle_frame(oid)
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = pose['position']
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = pose['orientation']
        return transform

    def publish_definition(self):
        with self.store.lock:
            key, message = self._applied_snapshot
            if key != (self.store.epoch, self.store.revision):
                message = snapshot(self.store.document, self.store.epoch, self.store.revision,
                                   self._obstacle_messages)
            self.snapshot_pub.publish(message)
            static = [self.transform(item['id'], item['pose'], message.header.stamp)
                      for item in self.store.document['obstacles'] if item['motion']['type'] == 'hold']
            if static:
                self.tf_static.sendTransform(static)
            markers = MarkerArray()
            delete = Marker()
            delete.action = Marker.DELETEALL
            markers.markers.append(delete)
            cache = {}
            for item, body in zip(self.store.document['obstacles'], message.obstacles):
                markers.markers.extend(self.obstacle_markers(item, body, cache))
            self._markers = cache
            self.marker_pub.publish(markers)
            self.tick(None)
            self.publish_document()

    def obstacle_markers(self, item, body, cache):
        """Markers of one obstacle, reused while the obstacle and its frame are."""
        frame = self.obstacle_frame(body.id)
        cached = self._markers.get(id(item))
        if cached is None or cached[0] is not item or cached[1] != frame or item['id'] != body.id:
            cached = (item, frame, [marker for part in body.parts for marker in self.part_markers(body.id, part)])
        cache[id(item)] = cached
        return cached[2]

    def part_markers(self, oid, part):
        geometry = part.geometry
        marker = Marker()
        marker.header.frame_id = self.obstacle_frame(oid)
        marker.header.stamp = rospy.Time(0)
        marker.ns = oid+'/'+part.id
        marker.id = 0
        marker.action = Marker.ADD
        marker.pose = part.pose
        marker.color = part.color
        marker.frame_locked = True
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0
        if geometry.type == 'box':
            marker.type = Marker.CUBE
            marker.scale = geometry.size
        elif geometry.type == 'sphere':
            marker.type = Marker.SPHERE
            marker.scale.x = marker.scale.y = marker.scale.z = 2*geometry.radius
        elif geometry.type in ('cylinder', 'capsule'):
            marker.type = Marker.CYLINDER
            marker.scale.x = marker.scale.y = 2*geometry.radius
            marker.scale.z = geometry.height
            if geometry.type == 'capsule':
                result = [marker] if geometry.height > 0 else []
                for index, direction in enumerate((-1, 1) if geometry.height else (1,)):
                    # The cylinder marker with another id, a sphere shape and its
                    # position moved to one end (built directly: a deep copy of a
                    # marker per cap made definitions of ring-heavy scenes slow).
                    q = marker.pose.orientation
                    delta = rotate([q.x, q.y, q.z, q.w], [0, 0, direction*geometry.height/2])
                    cap = Marker()
                    cap.header.frame_id = marker.header.frame_id
                    cap.header.stamp = marker.header.stamp
                    cap.ns = marker.ns
                    cap.id = index+1
                    cap.type = Marker.SPHERE
                    cap.action = marker.action
                    cap.pose = Pose()
                    cap.pose.position.x = marker.pose.position.x + delta[0]
                    cap.pose.position.y = marker.pose.position.y + delta[1]
                    cap.pose.position.z = marker.pose.position.z + delta[2]
                    cap.pose.orientation.x, cap.pose.orientation.y = q.x, q.y
                    cap.pose.orientation.z, cap.pose.orientation.w = q.z, q.w
                    cap.scale.x = cap.scale.y = cap.scale.z = 2*geometry.radius
                    cap.color = marker.color
                    cap.frame_locked = marker.frame_locked
                    result.append(cap)
                return result
        elif geometry.type == 'convex':
            marker.type = Marker.TRIANGLE_LIST
            marker.points = [geometry.vertices[i] for i in geometry.triangles]
        return [marker]

    def tick(self, _event):
        with self.store.lock:
            message = SceneState()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = self.store.document['frame']
            message.epoch, message.revision = self.store.epoch, self.store.revision
            message.playing, message.scene_time = self.store.playing, self.store.scene_time()
            transforms = []
            entries = self.state_entries()
            elapsed = self.store.scene_time()
            for item, constant in entries:
                if constant is not None:
                    message.obstacles.append(constant)
                    continue
                value = state(item, elapsed, self.store.playing)
                message.obstacles.append(state_message(value))
                transforms.append(self.transform(value['id'], value['pose'], message.header.stamp))
            if transforms:
                self.tf_dynamic.sendTransform(transforms)
            self.state_pub.publish(message)

    def state_entries(self):
        """Per document: each obstacle with its state message when it holds.

        A hold obstacle's state is its initial pose with zero twist whether or
        not the scene plays, so that message is built once per document and
        only moving obstacles are evaluated per tick.
        """
        document, entries = self._states
        if document is not self.store.document:
            document = self.store.document
            entries = [(item, state_message(state(item, 0.0, False)) if item['motion']['type'] == 'hold' else None)
                       for item in document['obstacles']]
            self._states = (document, entries)
        return entries

    def shutdown(self):
        if not self.online:
            return
        self.online = False
        self.publish_document()
        self.service.close()
        if self.simulation is not None:
            self.simulation.close()
        self.xrpc_runtime.close()


def main():
    rospy.init_node('scene_runtime')
    try:
        SceneNode()
        rospy.spin()
    except (SceneError, OSError, rospy.ROSException) as error:
        rospy.logfatal('Scene could not start: %s', error)
        raise SystemExit(1)
