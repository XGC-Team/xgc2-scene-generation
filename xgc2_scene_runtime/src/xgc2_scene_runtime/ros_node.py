"""ROS transport projections of the simulator-independent scene document."""

import copy
import hashlib
import json
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from xgc2_geometry_msgs.msg import (
    SceneSnapshot, SceneObstacle, ScenePart, SceneState, SceneObstacleState, SceneConsumerStatus,
)
from xgc2_geometry_msgs.srv import ApplyScene, SceneCommand, SceneCommandResponse

from .document import SceneError
from .store import MAX_DOCUMENT_BYTES, SceneStore, load, unique_object


def set_pose(message, value):
    message.position.x, message.position.y, message.position.z = value['position']
    message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = value['orientation']


def snapshot(document, epoch, revision):
    result = SceneSnapshot()
    result.header.stamp = rospy.Time.now()
    result.header.frame_id = document['frame']
    result.scene_id, result.epoch, result.revision = document['id'], epoch, revision
    for item in document['obstacles']:
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
        result.obstacles.append(body)
    return result


class SceneNode:
    def __init__(self):
        source = rospy.get_param('~scene_file')
        initial, source_digest = load(source)
        self.gazebo = bool(rospy.get_param('~gazebo', True))
        self.consumers = {}
        self.consumer_lock = threading.RLock()
        self.online = True
        self.snapshot_pub = rospy.Publisher('snapshot', SceneSnapshot, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher('state', SceneState, queue_size=1)
        self.document_pub = rospy.Publisher('document', String, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('markers', MarkerArray, queue_size=1, latch=True)
        self.tf_static = tf2_ros.StaticTransformBroadcaster()
        self.tf_dynamic = tf2_ros.TransformBroadcaster()
        self.frame_prefix = 'xgc_scene_' + hashlib.sha256(rospy.get_namespace().encode()).hexdigest()[:12]
        self.apply_service = None
        if self.gazebo:
            rospy.wait_for_service('gazebo/apply', timeout=30.0)
            self.apply_service = rospy.ServiceProxy('gazebo/apply', ApplyScene)
        self.store = SceneStore(initial, source, rospy.get_param('~save_directory', '') or None,
                                self.apply_scene, lambda: rospy.Time.now().to_sec(), source_digest=source_digest)
        self.status_sub = rospy.Subscriber('consumer_status', SceneConsumerStatus, self.consumer_status, queue_size=50)
        self.service = rospy.Service('command', SceneCommand, self.command)
        self.publish_definition()
        self.timer = rospy.Timer(rospy.Duration(1.0/30.0), self.tick, reset=True)
        rospy.on_shutdown(self.shutdown)

    def apply_scene(self, document, epoch, revision):
        if self.apply_service is None:
            return
        try:
            result = self.apply_service(snapshot(document, epoch, revision))
            if not result.success or result.epoch != epoch or result.applied_revision != revision:
                raise SceneError('Gazebo did not apply the scene: {}'.format(result.message))
        except rospy.ServiceException as error:
            self.gazebo_failure(epoch, revision, str(error))
            raise SceneError('Gazebo scene update failed: {}'.format(error))
        except SceneError as error:
            self.gazebo_failure(epoch, revision, str(error))
            raise
        with self.consumer_lock:
            self.consumers['gazebo'] = {'consumer': 'gazebo', 'epoch': epoch, 'revision': revision,
                                       'success': True, 'message': result.message}

    def gazebo_failure(self, epoch, revision, message):
        # A failed factory operation can leave a partially changed simulator.
        # The accepted document stays intact, but the old success must not mask this.
        with self.consumer_lock:
            self.consumers['gazebo'] = {'consumer': 'gazebo', 'epoch': epoch, 'revision': revision,
                                       'success': False, 'message': message}

    def envelope(self):
        result = self.store.envelope()
        with self.consumer_lock:
            result['consumers'] = copy.deepcopy(list(self.consumers.values()))
        result['online'] = self.online
        result['synchronized'] = all(item['success'] and item['epoch'] == result['epoch'] and item['revision'] == result['revision'] for item in result['consumers'])
        return result

    def publish_document(self):
        self.document_pub.publish(String(json.dumps(self.envelope(), ensure_ascii=False, allow_nan=False)))

    def consumer_status(self, message):
        # Store the latest reported version; mismatches remain visible, not promoted to success.
        with self.consumer_lock:
            self.consumers[message.consumer] = {'consumer': message.consumer, 'epoch': message.epoch,
                                                'revision': message.revision, 'success': message.success,
                                                'message': message.message}
        self.publish_document()

    def command(self, request):
        try:
            if len(request.command_json.encode('utf-8')) > MAX_DOCUMENT_BYTES:
                raise SceneError('Scene command exceeds the supported size')
            command = json.loads(request.command_json, object_pairs_hook=unique_object,
                                 parse_constant=lambda value: (_ for _ in ()).throw(SceneError('Nonfinite JSON value')))
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
        return SceneCommandResponse(success=result['success'], result_json=json.dumps(result, ensure_ascii=False, allow_nan=False))

    def obstacle_frame(self, oid):
        item = next(item for item in self.store.document['obstacles'] if item['id'] == oid)
        mode = 'fixed' if item['motion']['type'] == 'hold' else 'moving'
        # tf2 caches static transforms indefinitely. Switching motion modes must
        # not reuse the same child frame for both static and dynamic TF data.
        return '{}/{}/{}/{}'.format(self.frame_prefix, self.store.epoch, mode, oid)

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
            message = snapshot(self.store.document, self.store.epoch, self.store.revision)
            self.snapshot_pub.publish(message)
            static = [self.transform(item['id'], item['pose'], message.header.stamp)
                      for item in self.store.document['obstacles'] if item['motion']['type'] == 'hold']
            if static:
                self.tf_static.sendTransform(static)
            markers = MarkerArray()
            delete = Marker()
            delete.action = Marker.DELETEALL
            markers.markers.append(delete)
            for body in message.obstacles:
                for part in body.parts:
                    markers.markers.extend(self.part_markers(body.id, part))
            self.marker_pub.publish(markers)
            self.tick(None)
            self.publish_document()

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
                from .motion import rotate
                result = [marker] if geometry.height > 0 else []
                for index, direction in enumerate((-1, 1) if geometry.height else (1,)):
                    cap = copy.deepcopy(marker)
                    cap.id = index+1
                    cap.type = Marker.SPHERE
                    cap.scale.x = cap.scale.y = cap.scale.z = 2*geometry.radius
                    q = marker.pose.orientation
                    delta = rotate([q.x, q.y, q.z, q.w], [0, 0, direction*geometry.height/2])
                    cap.pose.position.x += delta[0]
                    cap.pose.position.y += delta[1]
                    cap.pose.position.z += delta[2]
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
            dynamic = {item['id'] for item in self.store.document['obstacles'] if item['motion']['type'] != 'hold'}
            for item in self.store.states():
                body = SceneObstacleState()
                body.id = item['id']
                set_pose(body.pose, item['pose'])
                body.twist.linear.x, body.twist.linear.y, body.twist.linear.z = item['linear']
                body.twist.angular.x, body.twist.angular.y, body.twist.angular.z = item['angular']
                message.obstacles.append(body)
                if body.id in dynamic:
                    transforms.append(self.transform(body.id, item['pose'], message.header.stamp))
            if transforms:
                self.tf_dynamic.sendTransform(transforms)
            self.state_pub.publish(message)

    def shutdown(self):
        self.online = False
        self.publish_document()


def main():
    rospy.init_node('scene_runtime')
    try:
        SceneNode()
        rospy.spin()
    except (SceneError, OSError, rospy.ROSException) as error:
        rospy.logfatal('Scene could not start: %s', error)
        raise SystemExit(1)
