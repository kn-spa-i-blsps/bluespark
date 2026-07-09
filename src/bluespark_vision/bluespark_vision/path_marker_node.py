"""
File: path_marker_node.py
Purpose: ROS 2 node that detects the path marker (an arrow-shaped marker on the
         seafloor) in the camera stream and publishes its position and pointing
         direction for the control layer.

         Subscribes to raw frames on /camera/image_raw, runs the classic
         HSV + contour detector (PathMarkerDetector), smooths the result over
         time with an exponential moving average (EMA), and publishes a
         VisionTarget on /path_marker/data.

         Like the other detectors, this node only reports what it sees; it does
         not command any motion, and it reads from the shared camera stream
         owned by camera_node.

Publishes:
    /path_marker/data   (bluespark_interfaces/VisionTarget)   source = "path_marker"
Subscribes:
    /camera/image_raw   (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from bluespark_interfaces.msg import VisionTarget
from .image_utils import ros_image_to_cv2
from .detectors.path_marker import PathMarkerDetector


class PathMarkerNode(Node):
    def __init__(self):
        super().__init__('path_marker_node')

        self.declare_parameter('hsv_h_min', 10)
        self.declare_parameter('hsv_h_max', 25)
        self.declare_parameter('hsv_s_min', 100)
        self.declare_parameter('hsv_s_max', 255)
        self.declare_parameter('hsv_v_min', 100)
        self.declare_parameter('hsv_v_max', 255)
        self.declare_parameter('min_area_fraction', 0.005)
        self.declare_parameter('min_aspect_ratio', 2.0)
        self.declare_parameter('ema_alpha', 0.7)

        self.detector = PathMarkerDetector(
            hsv_lower=(
                self.get_parameter('hsv_h_min').value,
                self.get_parameter('hsv_s_min').value,
                self.get_parameter('hsv_v_min').value,
            ),
            hsv_upper=(
                self.get_parameter('hsv_h_max').value,
                self.get_parameter('hsv_s_max').value,
                self.get_parameter('hsv_v_max').value,
            ),
            min_area_fraction=self.get_parameter('min_area_fraction').value,
            min_aspect_ratio=self.get_parameter('min_aspect_ratio').value,
        )

        self._alpha = self.get_parameter('ema_alpha').value
        self._ema_offset_x = 0.0
        self._ema_angle_deg = 0.0
        self._ema_confidence = 0.0
        self._ema_tip_direction = 0.0

        self.publisher = self.create_publisher(VisionTarget, '/path_marker/data', 10)
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10
        )

        self.get_logger().info('Path marker node ready, subscribing to /camera/image_raw')

    def _image_callback(self, msg: Image):
        frame = ros_image_to_cv2(msg)
        result = self.detector.detect(frame)

        if result.detected:
            self._ema_offset_x = (self._alpha * result.offset_x
                                  + (1 - self._alpha) * self._ema_offset_x)
            self._ema_angle_deg = (self._alpha * result.angle_deg
                                   + (1 - self._alpha) * self._ema_angle_deg)
            self._ema_confidence = (self._alpha * result.confidence
                                    + (1 - self._alpha) * self._ema_confidence)
            if result.tip_direction is not None:
                self._ema_tip_direction = (self._alpha * result.tip_direction
                                           + (1 - self._alpha) * self._ema_tip_direction)
        else:
            # Decay confidence when nothing is detected
            self._ema_confidence *= (1 - self._alpha)

        out = VisionTarget()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.source = 'path_marker'
        out.detected = result.detected
        out.offset_x = self._ema_offset_x
        out.angle_deg = self._ema_angle_deg
        out.confidence = self._ema_confidence
        out.area_fraction = result.area_fraction
        out.tip_direction = self._ema_tip_direction
        out.tip_direction_valid = result.tip_direction is not None

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PathMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()