"""
Code for testing ArUco marker detection using raw camera input
through simulation and through live camera feed
"""
import cv2
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ArucoLiveTest(Node):
    def __init__(self):
        super().__init__('aruco_live_test')

        self.bridge = CvBridge()

        # Match the dictionary used by the successful standalone test.
        self.dictionary = cv2.aruco.Dictionary_get(
            cv2.aruco.DICT_6X6_250
        )
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info('ArUco live detector started')
        self.get_logger().info('Camera topic: /camera/image_raw')
        self.get_logger().info('Dictionary: DICT_6X6_250')
        self.get_logger().info('Press Q in the image window to quit')

    def image_callback(self, msg):
        try:
            # Convert the ROS image into an OpenCV image.
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detect ArUco markers in the current camera frame.
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters
            )

            if ids is not None and len(ids) > 0:
                detected_ids = ids.flatten().tolist()

                cv2.aruco.drawDetectedMarkers(
                    frame, corners, ids
                )

                for marker_id, corner in zip(detected_ids, corners):
                    points = corner.reshape(4, 2).astype(int)
                    x, y = points[0]

                    cv2.putText(
                        frame,
                        f'ID: {marker_id}',
                        (x, max(y - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2
                    )

                status = f'DETECTED: {detected_ids}'
                color = (0, 255, 0)
            else:
                status = 'NO MARKER DETECTED'
                color = (0, 0, 255)

            cv2.putText(
                frame,
                status,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2
            )

            # Display the live feed with detection results.
            cv2.imshow('TurtleBot3 ArUco Live Test', frame)
            cv2.waitKey(1)

        except (CvBridgeError, cv2.error) as error:
            self.get_logger().error(str(error))

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLiveTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
