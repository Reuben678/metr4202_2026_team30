
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge

import cv2
import numpy as np


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter(
            'image_topic',
            '/camera/image_raw'
        )

        self.declare_parameter(
            'marker_size',
            0.10
        )

        self.image_topic = self.get_parameter(
            'image_topic'
        ).value

        self.marker_size = self.get_parameter(
            'marker_size'
        ).value

        # ---------------------------------------------------------
        # OpenCV / ROS setup
        # ---------------------------------------------------------

        self.bridge = CvBridge()

        # ArUco 6x6 dictionary.
        #
        # Change this to DICT_6X6_100, DICT_6X6_250,
        # DICT_6X6_1000, etc. if your marker uses another
        # dictionary.
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250
        )

        self.aruco_parameters = cv2.aruco.DetectorParameters()

        self.aruco_detector = cv2.aruco.ArucoDetector(
            self.aruco_dictionary,
            self.aruco_parameters
        )

        # ---------------------------------------------------------
        # Detected marker storage
        # ---------------------------------------------------------

        # Dictionary:
        #
        # {
        #     marker_id: {
        #         'id': marker_id,
        #         'last_seen': ROS time,
        #         'centre_x': pixel x,
        #         'centre_y': pixel y
        #     }
        # }
        #
        # This means that once a marker is detected, its ID is
        # retained by this node.

        self.detected_markers = {}

        # ---------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------

        self.image_subscriber = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # Publishes each newly detected ArUco ID.
        self.marker_id_publisher = self.create_publisher(
            Int32,
            '/aruco/detected_id',
            10
        )

        self.get_logger().info(
            'ArUco detector started'
        )

        self.get_logger().info(
            f'Subscribing to camera: {self.image_topic}'
        )

        self.get_logger().info(
            'Using ArUco dictionary: DICT_6X6_250'
        )

    # -------------------------------------------------------------
    # Camera callback
    # -------------------------------------------------------------

    def image_callback(self, msg):

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as error:
            self.get_logger().error(
                f'Could not convert camera image: {error}'
            )
            return

        # Convert to grayscale.
        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        # Detect markers.
        corners, ids, rejected = self.aruco_detector.detectMarkers(
            gray
        )

        # No markers detected.
        if ids is None:
            return

        # Flatten:
        #
        # [[23], [7]] -> [23, 7]
        ids = ids.flatten()

        for marker_index, marker_id in enumerate(ids):

            marker_id = int(marker_id)

            # -----------------------------------------------------
            # Calculate image centre of marker
            # -----------------------------------------------------

            marker_corners = corners[marker_index][0]

            centre_x = int(
                np.mean(marker_corners[:, 0])
            )

            centre_y = int(
                np.mean(marker_corners[:, 1])
            )

            # -----------------------------------------------------
            # Store marker
            # -----------------------------------------------------

            new_marker = marker_id not in self.detected_markers

            self.detected_markers[marker_id] = {
                'id': marker_id,
                'last_seen': self.get_clock().now(),
                'centre_x': centre_x,
                'centre_y': centre_y
            }

            # -----------------------------------------------------
            # Log detection
            # -----------------------------------------------------

            if new_marker:

                self.get_logger().info(
                    f'NEW ArUco marker detected: '
                    f'ID={marker_id}, '
                    f'pixel=({centre_x}, {centre_y})'
                )

                # Publish the ID.
                id_msg = Int32()
                id_msg.data = marker_id

                self.marker_id_publisher.publish(
                    id_msg
                )

            else:

                self.get_logger().debug(
                    f'ArUco marker {marker_id} '
                    f'is still visible'
                )

    # -------------------------------------------------------------
    # Helper function
    # -------------------------------------------------------------

    def get_detected_marker_ids(self):

        return list(
            self.detected_markers.keys()
        )


def main(args=None):

    rclpy.init(args=args)

    node = ArucoDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
