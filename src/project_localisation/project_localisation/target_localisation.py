# Receive corner, estimate calibrated poses, transform at image time and retain fused map records.

    #  Publish camera observations and timestamped map-frame target records
import json

import cv2
import rclpy
import math
import csv
import numpy as np

from pathlib import Path
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from collections import deque
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from project_localisation.marker_geometry import transform, fuse, update_record, estimate, square_points
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from rclpy.time import Time
from tf2_py import TransformException

LATCH = QoSProfile(depth = 1, durability = DurabilityPolicy.TRANSIENT_LOCAL)

def parameter(node, name, default):
    return node.declare_parameter(name, default).value

def second(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

def parts(tf):
    t = tf.transform.translation
    q = tf.transform.rotation
    return [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]

def send(pub, data):
    pub.publish(String(data=json.dumps(data, allow_nan=False)))

def save(path, data):
    path = Path(path).expanduser()
    path.parent.mkdir(parents = True, exist_ok = True)
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False), encoding='utf-8')
    temp.replace(path)

def run(cls):
    rclpy.init()
    node = cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destory_node()
        if rclpy.ok():
            rclpy.shutdown()


class Localizer(Node):
    def __init__(self):
        super().__init__('target_localizer')
        self.output = Path(parameter(self, 'output_dir', '~/localisation_runs'))
        self.output = self.output.expanduser()
        self.output.mkdir(parents=True, exist_ok=True)
        self.tf = Buffer(cache_time=Duration(seconds=20))
        self.listener = TransformListener(self.tf, self)
        self.queue = deque(maxlen=20)
        self.tracks, self.records = {}, {}
        self.raw_time = -1e9
        self.last_tf = None
        self.epoch = 0
        self.baseline = parameter(self, 'min_baseline', 0.15)
        self.scatter = parameter(self, 'max_scatter', 0.15)
        self.status = self.create_publisher(String, '/targets/state', LATCH)
        self.markers = self.create_publisher(MarkerArray,'/targets/markers', LATCH)
        self.info = None
        self.size = parameter(self, 'marker_size', 0.10)
        self.min_side = parameter(self, 'min_side_px', 25.0)
        self.max_range = parameter(self, 'max_range', 3.0)
        self.error_limit = parameter(self, 'max_reprojection', 2.0)
        info_topic = parameter(self, 'info_topic', '/camera/camera_info')
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(String, '/aruco/corners', self.receive, 20)
        self.create_service(Trigger, '/targets/reset', self.reset_service)
        self.create_timer(0.1, self.process)
        self.create_timer(1.0, self.publish)
        self.log_file = (self.output / 'observations.csv').open('w', newline='')
        self.writer = csv.writer(self.log_file)
        self.writer.writerow(['stamp', 'epoch', 'id', 'x', 'y', 'z', 'reprojection'])

    def now_s(self):
       return self.get_clock().now().nanoseconds * 1e-9

    def receive(self, msg):
       try:
           data = json.loads(msg.data)
           stamp = int(data['stamp_ns']) * 1e-9
           if not -0.05 <= self.now_s() - stamp <= 1.0 or self.info is None:
               return
           info = self.info
           if (data['frame'] != info.header.frame_id or not data['frame']
                   or data['width'] != info.width or data['height'] != info.height
                   or info.k[0] <= 0 or info.k[4] <= 0
                   or info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial')
                   or not isinstance(data['items'], list)):
               raise ValueError('CameraInfo does not match the image')
           K, D = np.array(info.k).reshape(3,3), np.array(info.d, dtype=float)
           if not np.isfinite(K).all() or not np.isfinite(D).all():
              raise ValueError('Non-finite calibration')
           self.raw_time = stamp
           items = []
           for item in data['items']:
              pose = estimate(item['corners'], self.size, K, D, info.width, info.height, self.min_side, self.max_range, self.error_limit)
              if pose is not None:
                 items.append(dict(id=int(item['id']), **pose))
           if items:
                 self.queue.append(dict(stamp_ns=data['stamp_ns'], frame=data['frame'], items=items))
       except (ValueError, KeyError, TypeError) as error:self.get_logger().warning(str(error))


    def on_info(self, msg):
       self.info = msg

    def reset(self):
       self.tracks.clear()
       self.records.clear()
       self.queue.clear()
       self.epoch += 1

    def reset_service(self, request, response):
       self.reset()
       response.success = True
       response.message = 'Tracks cleared; observe targets again'
       return response

    def process(self):
       try:
           latest = self.tf.lookup_transform('map', 'odom', Time())
           trans, q = parts(latest)
           yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1-2*(q[1]**2+q[2]**2))
           current = np.array([trans[0], trans[1], yaw])
           if self.last_tf is not None:
               delta = current - self.last_tf
               turn = abs(math.atan2(math.sin(delta[2]), math.cos(delta[2])))
               if np.linalg.norm(delta[:2]) > 0.25 or turn > 0.17:
                   self.reset()
                   self.get_logger().warning('Map jump: reobserve targets')
                   self.last_tf = current
               else:
                   self.last_tf = current
       except TransformException:
           return
       while self.queue:
           data = self.queue[0]
           t = data['stamp_ns'] * 1e-9
           if self.now_s() - t > 1.0:
              self.queue.popleft()
              continue
           stamp = Time(nanoseconds=int(data['stamp_ns']), clock_type=self.get_clock().clock_type)
           try:
               tf = self.tf.lookup_transform('map', data['frame'], stamp)
           except TransformException:
               break
           self.queue.popleft()
           trans, q = parts(tf)
           for item in data['items']:
              try:
                mid = int(item['id'])
                point = np.array(item['xyz'], dtype=float)
                error = float(item['reprojection'])
                if (mid < 0 or point.shape != (3,)
                        or not np.all(np.isfinite(point))
                        or not math.isfinite(error)
                        or not 0 <= error <= 2.0):
                    continue
                xyz = transform(point, trans, q)
                track = self.tracks.setdefault(mid, deque(maxlen=20))
                if track and t - track[-1][0] < 0.19:
                    continue
                if track and t - track[-1][0] > 30:
                    track.clear()
                track.append((t, xyz, np.array(trans)))
                self.writer.writerow([t, self.epoch, mid, *xyz.tolist(), error])
              except (ValueError, KeyError, TypeError):
                continue



    def publish(self):
        records = []
        array = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for mid, samples in sorted(self.tracks.items()):
            result = fuse(list(samples), min_baseline=self.baseline, max_scatter=self.scatter)
            if result is None:
                continue
            result = update_record(self.records.get(mid), result, samples[-1][0])
            self.records[mid] = result
            record = dict(id=mid, **result)
            records.append(record)
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id = 'target_labels', mid
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            xyz = result['xyz']
            marker.pose.position.x, marker.pose.position.y = xyz[:2]
            marker.pose.position.z = xyz[2] + 0.2
            marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0  if result['confirmed'] else 0.0
            marker.text = f"ID {mid}: " + ('confirmed' if result['confirmed'] else 'tentative')
            array.markers.append(marker)

        state = dict(frame='map', epoch=self.epoch,
                                              camera_age=max(0.0, self.now_s()-self.raw_time),
                                               targets=records)
        send(self.status, state)
        self.markers.publish(array)
        save(self.output/'target.json', state)
        self.log_file.flush()

def localizer_main():
    run(Localizer)



class Detector(Node):
    def __init__(self):
        super().__init__('marker_detector')
        self.size = parameter(self, 'marker_size', 0.10)
        square_points(self.size)
        name = parameter(self, 'dictionary', 'DICT_6X6_250')
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, name))
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.settings = cv2.aruco.DetectorParameters_create()
        else:
            self.settings = cv2.aruco.DetectorParameters()
        self.settings.cornerRefinementMethod = (
            cv2.aruco.CORNER_REFINE_SUBPIX)
        self.bridge = CvBridge()
        self.info = None
        self.last = -1.0
        self.error_limit = parameter(self, 'max_reprojection', 2.0)
        self.max_range = parameter(self, 'max_range', 3.0)
        self.min_side = parameter(self, 'min_side_px', 25.0)
        self.pub = self.create_publisher(String, '/targets/raw', 10)
        image_topic = parameter(self, 'image_topic', '/camera/image_raw')
        info_topic = parameter(self, 'info_topic', '/camera/camera_info')
        self.create_subscription(CameraInfo, info_topic,
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, image_topic,
                                 self.on_image, qos_profile_sensor_data)

    def on_info(self, msg):
        self.info = msg

    def on_image(self, msg):
        stamp_ns = (msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec)
        t = stamp_ns * 1e-9
        if t < self.last:
            self.last = -1.0
        if t - self.last < 0.2 or self.info is None:
            return
        self.last = t
        info = self.info
        if (info.width != msg.width or info.height != msg.height
                or info.header.frame_id != msg.header.frame_id
                or info.k[0] <= 0
                or info.distortion_model not in
                ('', 'plumb_bob', 'rational_polynomial')):
            self.get_logger().error('CameraInfo mismatch/calibration error')
            return
        try:
            grey = self.bridge.imgmsg_to_cv2(msg, 'mono8')
            K = np.array(info.k).reshape(3, 3)
            D = np.array(info.d, dtype=float)
            if hasattr(cv2.aruco, 'ArucoDetector'):
                detector = cv2.aruco.ArucoDetector(self.dictionary, self.settings)
                corners, ids, _ = detector.detectMarkers(grey)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(
                    grey, self.dictionary, parameters=self.settings)
            items = []
            if ids is not None:
                    for corner, mid in zip(corners, ids.flatten()):
                       pose = self.estimate(corner, K, D, msg.width,
                                         msg.height)
                       if pose is not None:
                           items.append(dict(id=int(mid), **pose))

            send(self.pub, dict(stamp_ns=stamp_ns, frame=msg.header.frame_id, items=items))
        except (cv2.error, ValueError) as exc:
            self.get_logger().error(str(exc))






    def estimate(self, corners, K, D, width, height):
        return estimate(corners, self.size, K, D, width, height,
                    self.min_side, self.max_range, self.error_limit)

def detector_main():
    run(Detector)

































































































