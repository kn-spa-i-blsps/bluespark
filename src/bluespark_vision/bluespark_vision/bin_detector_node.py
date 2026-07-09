"""
File: bin_detector_node.py
Purpose: ROS 2 node that detects the bin (a colored container on the seafloor)
         and the symbol inside it (flame / drop) in the camera stream, and
         publishes its position and identified symbol for the control layer.

         Subscribes to raw frames on /camera/image_raw, runs the classic
         HSV / template detector (BinDetector), smooths the geometry over time
         with an exponential moving average (EMA), stabilizes the symbol with a
         majority vote over a sliding window, and publishes a VisionTarget on
         /bin/data.

         Like the other detectors, this node only reports what it sees; it does
         not command any motion, and it reads from the shared camera stream
         owned by camera_node.

Publishes:
    /bin/data   (bluespark_interfaces/VisionTarget)   source = "bin"
Subscribes:
    /camera/image_raw   (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from bluespark_interfaces.msg import VisionTarget
from .image_utils import ros_image_to_cv2
from .detectors.bin_detector import (
    BinDetector, BinDetectorConfig, SymbolDetectionMethod
)


class BinDetectorNode(Node):
    def __init__(self):
        super().__init__('bin_detector_node')

        self.declare_parameter('symbol_method', 'hsv')
        self.declare_parameter('bin_hsv_h_min', 100)
        self.declare_parameter('bin_hsv_h_max', 130)
        self.declare_parameter('bin_hsv_s_min', 80)
        self.declare_parameter('bin_hsv_s_max', 255)
        self.declare_parameter('bin_hsv_v_min', 30)
        self.declare_parameter('bin_hsv_v_max', 150)
        self.declare_parameter('flame_hsv_h_min', 0)
        self.declare_parameter('flame_hsv_h_max', 25)
        self.declare_parameter('flame_hsv_s_min', 100)
        self.declare_parameter('flame_hsv_s_max', 255)
        self.declare_parameter('flame_hsv_v_min', 150)
        self.declare_parameter('flame_hsv_v_max', 255)
        self.declare_parameter('drop_hsv_h_min', 140)
        self.declare_parameter('drop_hsv_h_max', 170)
        self.declare_parameter('drop_hsv_s_min', 80)
        self.declare_parameter('drop_hsv_s_max', 255)
        self.declare_parameter('drop_hsv_v_min', 150)
        self.declare_parameter('drop_hsv_v_max', 255)
        self.declare_parameter('flame_template_path', '')
        self.declare_parameter('drop_template_path', '')
        self.declare_parameter('template_threshold', 0.7)
        self.declare_parameter('min_bin_area_fraction', 0.01)
        self.declare_parameter('ema_alpha', 0.7)
        self.declare_parameter('vote_window', 10)

        method_str = self.get_parameter('symbol_method').value
        method = (SymbolDetectionMethod.TEMPLATE
                  if method_str == 'template'
                  else SymbolDetectionMethod.HSV)

        config = BinDetectorConfig(
            bin_hsv_lower=(
                self.get_parameter('bin_hsv_h_min').value,
                self.get_parameter('bin_hsv_s_min').value,
                self.get_parameter('bin_hsv_v_min').value,
            ),
            bin_hsv_upper=(
                self.get_parameter('bin_hsv_h_max').value,
                self.get_parameter('bin_hsv_s_max').value,
                self.get_parameter('bin_hsv_v_max').value,
            ),
            flame_hsv_lower=(
                self.get_parameter('flame_hsv_h_min').value,
                self.get_parameter('flame_hsv_s_min').value,
                self.get_parameter('flame_hsv_v_min').value,
            ),
            flame_hsv_upper=(
                self.get_parameter('flame_hsv_h_max').value,
                self.get_parameter('flame_hsv_s_max').value,
                self.get_parameter('flame_hsv_v_max').value,
            ),
            drop_hsv_lower=(
                self.get_parameter('drop_hsv_h_min').value,
                self.get_parameter('drop_hsv_s_min').value,
                self.get_parameter('drop_hsv_v_min').value,
            ),
            drop_hsv_upper=(
                self.get_parameter('drop_hsv_h_max').value,
                self.get_parameter('drop_hsv_s_max').value,
                self.get_parameter('drop_hsv_v_max').value,
            ),
            flame_template_path=self.get_parameter('flame_template_path').value,
            drop_template_path=self.get_parameter('drop_template_path').value,
            template_match_threshold=self.get_parameter('template_threshold').value,
            min_bin_area_fraction=self.get_parameter('min_bin_area_fraction').value,
            symbol_method=method,
        )

        self.detector = BinDetector(config)

        self._alpha = self.get_parameter('ema_alpha').value
        self._vote_window = self.get_parameter('vote_window').value
        self._ema_offset_x = 0.0
        self._ema_offset_y = 0.0
        self._ema_area_fraction = 0.0
        self._ema_symbol_confidence = 0.0
        self._last_symbol = ''
        self._vote_history = []

        self.publisher = self.create_publisher(VisionTarget, '/bin/data', 10)
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10
        )

        self.get_logger().info(
            f'Bin detector node ready (method={method_str}), '
            f'subscribing to /camera/image_raw'
        )

    def _image_callback(self, msg: Image):
        frame = ros_image_to_cv2(msg)
        result = self.detector.detect(frame)

        if result.detected:
            self._ema_offset_x = (self._alpha * result.offset_x
                                  + (1 - self._alpha) * self._ema_offset_x)
            self._ema_offset_y = (self._alpha * result.offset_y
                                  + (1 - self._alpha) * self._ema_offset_y)
            self._ema_area_fraction = (self._alpha * result.area_fraction
                                       + (1 - self._alpha) * self._ema_area_fraction)
            self._ema_symbol_confidence = (
                self._alpha * result.symbol_confidence
                + (1 - self._alpha) * self._ema_symbol_confidence
            )

            vote = result.symbol or ''
            self._vote_history.append(vote)
            if len(self._vote_history) > self._vote_window:
                self._vote_history.pop(0)

            counts = {'flame': 0, 'drop': 0, '': 0}
            for v in self._vote_history:
                counts[v] = counts.get(v, 0) + 1

            self._last_symbol = max(counts, key=counts.__getitem__)
        else:
            self._ema_area_fraction *= (1 - self._alpha)
            self._ema_symbol_confidence *= (1 - self._alpha)
            self._vote_history.clear()
            self._last_symbol = ''

        out = VisionTarget()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.source = 'bin'
        out.detected = result.detected
        out.offset_x = self._ema_offset_x
        out.offset_y = self._ema_offset_y
        out.area_fraction = self._ema_area_fraction
        out.symbol = self._last_symbol
        out.confidence = self._ema_symbol_confidence
        out.bbox_x = result.bbox[0]
        out.bbox_y = result.bbox[1]
        out.bbox_w = result.bbox[2]
        out.bbox_h = result.bbox[3]

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = BinDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()