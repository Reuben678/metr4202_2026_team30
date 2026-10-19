import csv
import json
import math
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from project_localisation.marker_geometry import (
    estimate, fuse, rotation, square_points, transform, update_record,
)

LATCH = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def parameter(node, name, default):
    return node.declare_parameter(name, default).value


def parts(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    return [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]


def save(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False),
                         encoding='utf-8')
    temporary.replace(path)


class Localizer(Node):
    def __init__(self):
        super().__init__('target_localizer')
        self.size = parameter(self, 'marker_size', 0.10)
        square_points(self.size)
        self.min_side = parameter(self, 'min_side_px', 25.0)
        self.max_range = parameter(self, 'max_range', 3.0)
        self.error_limit = parameter(self, 'max_reprojection', 2.0)
        self.baseline = parameter(self, 'min_baseline', 0.15)
        self.scatter = parameter(self, 'max_scatter', 0.15)
        self.max_age = parameter(self, 'max_image_age', 1.0)
        self.fresh_for = parameter(self, 'fresh_for', 10.0)
<<<<<<< Updated upstream:src/project_localisation/target_localisation.py
        # Preserve the teammate-ID contract by default. Independent tests can
        # disable this gate because this node already decodes the marker ID.
        self.require_announced_ids = parameter(self, 'require_announced_ids', True)
=======

        # Decode marker IDs locally by default, as in the working live test.
        # Set true only when a separate node publishes /aruco/detected_id.
        self.require_announced_ids = parameter(self, 'require_announced_ids', False)
>>>>>>> Stashed changes:project_localisation/project_localisation/target_localisation.py
        self.track_capacity = parameter(self, 'track_capacity', 100)
        if self.track_capacity < 8:
            raise ValueError('track_capacity must be at least 8')
        limits = [self.min_side, self.max_range, self.error_limit,
                  self.baseline, self.scatter, self.max_age, self.fresh_for]
        if not all(math.isfinite(x) and x > 0 for x in limits):
            raise ValueError('All quality and timing limits must be positive')

        self.bridge = CvBridge()
        self.dictionary_name = parameter(self, 'aruco_dictionary', 'DICT_6X6_250')
        if not hasattr(cv2, 'aruco') or not self.dictionary_name.startswith('DICT_6X6_'):
            raise ValueError('A 6x6 ArUco dictionary and OpenCV aruco are required')
        if not hasattr(cv2.aruco, self.dictionary_name):
            raise ValueError('Unknown ArUco dictionary: ' + self.dictionary_name)
        # Match the API used by the known-working standalone detector.
        # Keep fallbacks for OpenCV versions that expose the newer API only.
        if hasattr(cv2.aruco, 'Dictionary_get'):
            self.dictionary = cv2.aruco.Dictionary_get(
                getattr(cv2.aruco, self.dictionary_name))
        else:
            self.dictionary = cv2.aruco.getPredefinedDictionary(
                getattr(cv2.aruco, self.dictionary_name))
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.detector_parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.detector_parameters = cv2.aruco.DetectorParameters()
        self.detector_parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = (cv2.aruco.ArucoDetector(self.dictionary, self.detector_parameters)
                         if hasattr(cv2.aruco, 'ArucoDetector') else None)

        self.info = None
        self.info_signature = None
        self.announced_ids = set()
        self.queue = deque(maxlen=20)
        self.tracks, self.records = {}, {}
        self.dirty = set()
        self.epoch = 0
        self.last_tf = None
        self.raw_time = None
        self.last_image_ns = -1
        self.last_clock = self.now_s()
        self.tf = Buffer(cache_time=Duration(seconds=20))
        self.listener = TransformListener(self.tf, self)
        self._visible_marker_ids = set()

        base = Path(parameter(self, 'output_dir', '~/localisation_runs')).expanduser()
        self.output = base / datetime.now().strftime('run_%Y%m%d_%H%M%S_%f')
        self.output.mkdir(parents=True, exist_ok=False)
        self.log_file = (self.output / 'observations.csv').open('w', newline='')
        self.writer = csv.writer(self.log_file)
        self.writer.writerow(['stamp', 'epoch', 'id', 'x', 'y', 'z',
                              'reprojection', 'range_m', 'view_angle_deg'])

        self.status = self.create_publisher(String, '/targets/state', LATCH)
        self.markers = self.create_publisher(String, '/targets/markers', LATCH)
        self.create_subscription(
            Int32, parameter(self, 'id_topic', '/aruco/detected_id'), self.on_id, 20)
        self.create_subscription(
            CameraInfo, parameter(self, 'info_topic', '/camera/camera_info'),
            self.on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, parameter(self, 'image_topic', '/camera/image_raw'),
            self.on_image, qos_profile_sensor_data)
        self.create_service(Trigger, '/targets/reset', self.reset_service)
        self.create_timer(0.1, self.process)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f'Phase 8 output directory: {self.output}')
        if self.require_announced_ids:
            self.get_logger().info('Start this node before the one-shot ID detector.')
        else:
            self.get_logger().info('Using IDs decoded locally from each image.')

        self.marker_viz_pub = self.create_publisher(
            MarkerArray,
            '/target_markers',
            10
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def on_id(self, msg):
        mid = int(msg.data)
        if 0 <= mid < len(self.dictionary.bytesList):
            self.announced_ids.add(mid)
        else:
            self.warn('Received an ID outside the configured dictionary')

    def on_info(self, msg):
        signature = (msg.header.frame_id, msg.width, msg.height,
                     tuple(msg.k), tuple(msg.d), msg.distortion_model,
                     msg.binning_x, msg.binning_y, msg.roi.x_offset, msg.roi.y_offset)
        if self.info_signature is not None and signature != self.info_signature:
            self.reset('Camera calibration changed')
        self.info, self.info_signature = msg, signature

    def check_clock(self):
        now = self.now_s()
        if now < self.last_clock - 0.05:
            self.reset('ROS clock moved backwards')
            self.raw_time, self.last_image_ns = None, -1
        self.last_clock = now
        return now

    
    def on_image(self, msg):
        # Use the local ROS clock at image arrival for freshness and timing.
        # Keep the original image stamp for timestamped TF lookups.
        now = self.check_clock()
        stamp_ns = (
            msg.header.stamp.sec * 10**9
            + msg.header.stamp.nanosec
        )

        if stamp_ns <= 0:
            self.warn('Image has an invalid zero timestamp')
            return

        # Keep rejecting duplicate or out-of-order camera frames.
        if stamp_ns <= self.last_image_ns:
            return

        if self.info is None:
            self.warn('Waiting for CameraInfo')
            return

        try:
            info = self.info
            if (not msg.header.frame_id
                    or msg.header.frame_id != info.header.frame_id
                    or msg.width != info.width
                    or msg.height != info.height
                    or info.distortion_model not in (
                        '', 'plumb_bob', 'rational_polynomial'
                    )
                    or info.binning_x > 1
                    or info.binning_y > 1
                    or info.roi.x_offset
                    or info.roi.y_offset):
                raise ValueError(
                    'Use matching raw Image and full-resolution CameraInfo'
                )

            K = np.asarray(info.k).reshape(3, 3)
            D = np.asarray(info.d)

            if (not np.isfinite(K).all()
                    or not np.isfinite(D).all()
                    or min(K[0, 0], K[1, 1]) <= 0):
                raise ValueError('Invalid camera calibration')

            # Track camera freshness using arrival time, not the header stamp.
            self.raw_time = now

            # Limit processing rate to about 10 FPS using camera timestamps.
            if (self.last_image_ns >= 0
                    and stamp_ns - self.last_image_ns < 100_000_000):
                return

            self.last_image_ns = stamp_ns

<<<<<<< Updated upstream:src/project_localisation/target_localisation.py
            gray = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='mono8'
            )

=======
            # Match the working live test's image path: decode BGR8, then
            # convert to grayscale before ArUco detection.
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Use the same detectMarkers call as the working standalone test
            # when available; support newer OpenCV's ArucoDetector as well.
>>>>>>> Stashed changes:project_localisation/project_localisation/target_localisation.py
            if self.detector is not None:
                corners, ids, _ = self.detector.detectMarkers(gray)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(
<<<<<<< Updated upstream:src/project_localisation/target_localisation.py
                    gray,
                    self.dictionary,
                    parameters=self.detector_parameters,
                )
=======
                    gray, self.dictionary, parameters=self.detector_parameters)
>>>>>>> Stashed changes:project_localisation/project_localisation/target_localisation.py

            if ids is None:
                return

            flat_ids = ids.reshape(-1).tolist()
            items = []

            for mid, corner in zip(flat_ids, corners):
                if flat_ids.count(mid) != 1:
                    continue

                pose = estimate(
                    corner, self.size, K, D,
                    msg.width, msg.height,
                    self.min_side, self.max_range,
                    self.error_limit,
                )

                if pose is not None:
                    items.append(dict(id=int(mid), **pose))

            if items:
                self.queue.append({
                    'stamp_ns': stamp_ns,
                    'received_s': now,
                    'frame': msg.header.frame_id,
                    'items': items,
                })

        except (ValueError, TypeError, CvBridgeError, cv2.error) as error:
            self.warn(str(error))

    def reset(self, reason='Manual reset'):
        self.tracks.clear()
        self.records.clear()
        self.queue.clear()
        self.dirty.clear()
        self.last_tf = None
        # A calibration change must allow the next image to be processed again.
        self.last_image_ns = -1
        self.epoch += 1
        # Keep announced IDs: the teammate publishes each ID only once.
        self.get_logger().info(reason + '; position evidence cleared')

    def reset_service(self, request, response):
        self.reset()
        self.publish()
        response.success, response.message = True, 'Positions cleared; IDs retained'
        return response

    def process(self):
        now = self.check_clock()
        try:
            trans, q = parts(self.tf.lookup_transform('map', 'odom', Time()))
            R = rotation(q)
            current = np.array([trans[0], trans[1], math.atan2(R[1, 0], R[0, 0])])
            if not np.isfinite(current).all():
                raise ValueError('Non-finite map transform')
            if self.last_tf is not None:
                delta = current - self.last_tf
                turn = abs(math.atan2(math.sin(delta[2]), math.cos(delta[2])))
                if np.linalg.norm(delta[:2]) > 0.25 or turn > 0.17:
                    self.reset('Map transform jumped; reobserve targets')
            self.last_tf = current
        except (TransformException, ValueError) as error:
            self.warn('Map/odom jump monitoring unavailable: ' + str(error))

        for _ in range(len(self.queue)):
            data = self.queue.popleft()

            # Use local arrival time for freshness and observation timing.
            t = data['received_s']

            if not -0.05 <= now - t <= self.max_age:
                if self.require_announced_ids and any(
                        x['id'] not in self.announced_ids for x in data['items']):
                    self.warn('Visible ID was not announced. Restart project_aruco after this node.')
                continue
            approved = [x for x in data['items'] if (
                not self.require_announced_ids or x['id'] in self.announced_ids)]
            if not approved:
                self.queue.append(data)
                continue
            stamp = Time(nanoseconds=data['stamp_ns'], clock_type=self.get_clock().clock_type)
            try:
                trans, q = parts(self.tf.lookup_transform('map', data['frame'], stamp))
                camera = np.asarray(trans, dtype=float)
                if not np.isfinite(camera).all():
                    raise ValueError('Non-finite camera transform')
                for item in approved:
                    mid = item['id']
                    xyz = transform(item['xyz'], trans, q)
                    if not np.isfinite(xyz).all() or not 0 <= item['reprojection'] <= self.error_limit:
                        continue
                    track = self.tracks.setdefault(mid, deque(maxlen=self.track_capacity))
                    if track and t - track[-1][0] < 0.19:
                        continue
                    if track and t - track[-1][0] > 30:
                        track.clear()
                    track.append((t, xyz, camera))
                    self.dirty.add(mid)
                    self.writer.writerow([t, self.epoch, mid, *xyz.tolist(),
                                          item['reprojection'], item['range_m'],
                                          item['view_angle_deg']])
            except TransformException:
                self.queue.append(data)
                continue
            except ValueError as error:
                self.warn(str(error))
                continue
            pending = [x for x in data['items'] if (
                self.require_announced_ids and x['id'] not in self.announced_ids)]
            if pending:
                self.queue.append(dict(data, items=pending))

    def publish_marker_visualization(self):
        msg = MarkerArray()
        now = self.get_clock().now().to_msg()

        visible_ids = {
            mid for mid, record in self.records.items()
            if record.get('confirmed', False)
        }

        # Remove markers that are no longer confirmed.
        for mid in getattr(self, '_visible_marker_ids', set()) - visible_ids:
            for marker_id in (mid * 2, mid * 2 + 1):
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = now
                marker.ns = 'detected_targets'
                marker.id = marker_id
                marker.action = Marker.DELETE
                msg.markers.append(marker)

        for mid in sorted(visible_ids):
            record = self.records[mid]
            x, y, z = record['xyz']

            # Sphere showing the target position.
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = now
            sphere.ns = 'detected_targets'
            sphere.id = mid * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD

            sphere.pose.position.x = float(x)
            sphere.pose.position.y = float(y)
            sphere.pose.position.z = float(z)
            sphere.pose.orientation.w = 1.0

            sphere.scale.x = 0.12
            sphere.scale.y = 0.12
            sphere.scale.z = 0.12

            sphere.color.r = 0.1
            sphere.color.g = 1.0
            sphere.color.b = 0.2
            sphere.color.a = 1.0

            msg.markers.append(sphere)

            # Text label above the sphere.
            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = 'detected_targets'
            label.id = mid * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD

            label.pose.position.x = float(x)
            label.pose.position.y = float(y)
            label.pose.position.z = float(z) + 0.15
            label.pose.orientation.w = 1.0

            label.scale.z = 0.12
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'Marker {mid}'

            msg.markers.append(label)

        self.marker_viz_pub.publish(msg)
        self._visible_marker_ids = visible_ids

    def publish(self, ros_output=True):
        # Update target position estimates from new observations.
        for mid in sorted(self.dirty):
            samples = list(self.tracks[mid])
            result = fuse(
                samples,
                min_baseline=self.baseline,
                max_scatter=self.scatter
            )
            if result is not None:
                self.records[mid] = update_record(
                    self.records.get(mid),
                    result,
                    samples[-1][0]
                )

        self.dirty.clear()

        now = self.now_s()
        records = []

        # Clear previous RViz markers before publishing the current set.
        array = MarkerArray(
            markers=[Marker(action=Marker.DELETEALL)]
        )

        for mid, result in sorted(self.records.items()):
            age = max(0.0, now - result['last_seen'])
<<<<<<< Updated upstream:src/project_localisation/target_localisation.py
            records.append(dict(id=mid, **result, age_s=age, fresh=age <= self.fresh_for))
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id = 'target_labels', mid
            marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.pose.position.x, marker.pose.position.y = result['xyz'][:2]
            marker.pose.position.z = result['xyz'][2] + 0.2
            marker.scale.z = 0.15
            marker.color.a, marker.color.r = 1.0, 1.0
            marker.color.g = 1.0 if result['confirmed'] else 0.0
            marker.text = f'ID {mid}: ' + ('confirmed' if result['confirmed'] else 'tentative')
            array.markers.append(marker)
        state = dict(frame='map', epoch=self.epoch, run_id=self.output.name,
                     stamp=now, camera_age=None if self.raw_time is None else max(0.0, now-self.raw_time),
                     announced_ids=sorted(self.announced_ids), targets=records,
                     require_announced_ids=self.require_announced_ids,
                     pending_observations=len(self.queue),
                     marker_size=self.size, dictionary=self.dictionary_name)
        if ros_output:
            self.status.publish(String(data=json.dumps(state, allow_nan=False)))
<<<<<<< Updated upstream:src/project_localisation/target_localisation.py
            self.markers.publish(array)
=======

            marker_output_json = json.dumps({
                "targets": [
                    {
                        "id": mid,
                        "xyz": result["xyz"]
=======
            fresh = age <= self.fresh_for

            records.append(dict(
                id=mid,
                **result,
                age_s=age,
                fresh=fresh
            ))

            x, y, z = result['xyz']
            confirmed = result['confirmed']

            # Sphere representing the estimated target position.
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'detected_targets'
            sphere.id = mid * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD

            sphere.pose.position.x = float(x)
            sphere.pose.position.y = float(y)
            sphere.pose.position.z = float(z)
            sphere.pose.orientation.w = 1.0

            sphere.scale.x = 0.12
            sphere.scale.y = 0.12
            sphere.scale.z = 0.12

            # Green = confirmed, yellow = tentative.
            sphere.color.r = 0.1 if confirmed else 1.0
            sphere.color.g = 1.0
            sphere.color.b = 0.1 if confirmed else 0.0
            sphere.color.a = 1.0

            # Fade stale targets so old detections are distinguishable.
            if not fresh:
                sphere.color.a = 0.35

            array.markers.append(sphere)

            # Text label positioned above the sphere.
            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = 'target_labels'
            label.id = mid * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD

            label.pose.position.x = float(x)
            label.pose.position.y = float(y)
            label.pose.position.z = float(z) + 0.2
            label.pose.orientation.w = 1.0

            label.scale.z = 0.15
            label.color.r = 1.0
            label.color.g = 1.0 if confirmed else 0.7
            label.color.b = 0.0
            label.color.a = 1.0 if fresh else 0.35

            status = 'confirmed' if confirmed else 'tentative'
            if not fresh:
                status += ' (stale)'

            label.text = f'ID {mid}: {status}'
            array.markers.append(label)

        # Publish target state and marker data.
        state = dict(
            frame='map',
            epoch=self.epoch,
            run_id=self.output.name,
            stamp=now,
            camera_age=(
                None if self.raw_time is None
                else max(0.0, now - self.raw_time)
            ),
            announced_ids=sorted(self.announced_ids),
            targets=records,
            require_announced_ids=self.require_announced_ids,
            pending_observations=len(self.queue),
            marker_size=self.size,
            dictionary=self.dictionary_name
        )

        if ros_output:
            self.status.publish(
                String(data=json.dumps(state, allow_nan=False))
            )

            marker_output_json = json.dumps({
                'targets': [
                    {
                        'id': mid,
                        'xyz': result['xyz']
>>>>>>> Stashed changes:project_localisation/project_localisation/target_localisation.py
                    }
                    for mid, result in sorted(self.records.items())
                ]
            }, allow_nan=False)

<<<<<<< Updated upstream:src/project_localisation/target_localisation.py
            self.markers.publish(String(data=marker_output_json))

>>>>>>> Stashed changes:project_localisation/project_localisation/target_localisation.py
=======
            self.markers.publish(
                String(data=marker_output_json)
            )

            # Publish visualization markers for RViz2.
            self.marker_viz_pub.publish(array)

>>>>>>> Stashed changes:project_localisation/project_localisation/target_localisation.py
        save(self.output / 'targets.json', state)
        self.log_file.flush()


    def destroy_node(self):
        if hasattr(self, 'log_file') and not self.log_file.closed:
            try:
                self.publish(ros_output=rclpy.ok())
            finally:
                self.log_file.close()
        return super().destroy_node()


def localizer_main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Localizer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    localizer_main()
