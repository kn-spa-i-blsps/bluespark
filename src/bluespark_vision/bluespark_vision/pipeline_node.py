"""
File: pipeline_node.py
Purpose: ROS 2 node that detects the pipeline (a bright line on the seafloor)
         in the camera stream and publishes its position for the control layer.

         Subscribes to raw frames on /camera/image_raw, runs the classic
         HSV + Hough line detector (PipelineDetector), smooths the result over
         time with an exponential moving average (EMA), and publishes a
         VisionTarget on /pipeline/data.

         This node only reports what it sees (offset, angle, confidence); it
         does not command any motion. It is one of several detectors that read
         from the shared camera stream, so the camera is owned by camera_node,
         not by this node.

Publishes:
    /pipeline/data   (bluespark_interfaces/VisionTarget)   source = "pipeline"
Subscribes:
    /camera/image_raw   (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from bluespark_interfaces.msg import VisionTarget
from .image_utils import ros_image_to_cv2
from .detectors.pipeline import PipelineDetector


class PipelineNode(Node):
    def __init__(self):
        super().__init__('pipeline_node')

        self.declare_parameter('hsv_h_min', 0)
        self.declare_parameter('hsv_h_max', 180)
        self.declare_parameter('hsv_s_min', 0)
        self.declare_parameter('hsv_s_max', 35)
        self.declare_parameter('hsv_v_min', 200)
        self.declare_parameter('hsv_v_max', 255)
        self.declare_parameter('hough_threshold', 50)
        self.declare_parameter('hough_min_line_length', 80)
        self.declare_parameter('hough_max_line_gap', 30)
        self.declare_parameter('ema_alpha', 0.7)

        self.detector = PipelineDetector(
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
            hough_threshold=self.get_parameter('hough_threshold').value,
            hough_min_line_length=self.get_parameter('hough_min_line_length').value,
            hough_max_line_gap=self.get_parameter('hough_max_line_gap').value,
        )

        self._alpha = self.get_parameter('ema_alpha').value
        self._ema_offset_x = 0.0
        self._ema_angle_deg = 0.0
        self._ema_confidence = 0.0

        self.publisher = self.create_publisher(VisionTarget, '/pipeline/data', 10)
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, 10
        )

        self.get_logger().info('Pipeline node ready, subscribing to /camera/image_raw')

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
        else:
            # Decay confidence when nothing is detected
            self._ema_confidence *= (1 - self._alpha)

        out = VisionTarget()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.source = 'pipeline'
        out.detected = result.detected
        out.offset_x = self._ema_offset_x
        out.angle_deg = self._ema_angle_deg
        out.confidence = self._ema_confidence
        # Pipeline uses only the shared geometry fields; the rest stay at defaults.
        # Line width is carried in bbox_w so no detector information is lost.
        out.bbox_w = result.line_width_px

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PipelineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()