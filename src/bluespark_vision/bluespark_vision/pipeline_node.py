import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from picamera2 import Picamera2
from bluespark_interfaces.msg import PipelineData
from std_msgs.msg import Header


class PipelineDetectorNode(Node):
    def __init__(self):
        super().__init__('pipeline_detector_node')

        # Parameters
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('timer_period', 0.1)
        self.declare_parameter('hsv_h_min', 0)
        self.declare_parameter('hsv_h_max', 180)
        self.declare_parameter('hsv_s_min', 0)
        self.declare_parameter('hsv_s_max', 50)
        self.declare_parameter('hsv_v_min', 180)
        self.declare_parameter('hsv_v_max', 255)
        self.declare_parameter('hough_threshold', 80)
        self.declare_parameter('hough_min_line_length', 100)
        self.declare_parameter('hough_max_line_gap', 20)

        self.width = self.get_parameter('camera_width').value
        self.height = self.get_parameter('camera_height').value

        # Publisher
        self.publisher = self.create_publisher(PipelineData, '/pipeline/data', 10)

        # Camera
        self.cam = Picamera2()
        config = self.cam.create_preview_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"}
        )
        self.cam.configure(config)
        self.cam.start()

        # Timer
        timer_period = self.get_parameter('timer_period').value
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('Pipeline detector node ready.')

    def timer_callback(self):
        frame = self.cam.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        msg = self._detect_pipeline(frame_bgr)
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_bottom'

        self.publisher.publish(msg)

    def _detect_pipeline(self, frame: np.ndarray) -> PipelineData:
        msg = PipelineData()
        msg.detected = False
        msg.offset_x = 0.0
        msg.angle_deg = 0.0
        msg.confidence = 0.0
        msg.line_width_px = 0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower = np.array([
            self.get_parameter('hsv_h_min').value,
            self.get_parameter('hsv_s_min').value,
            self.get_parameter('hsv_v_min').value,
        ])
        upper = np.array([
            self.get_parameter('hsv_h_max').value,
            self.get_parameter('hsv_s_max').value,
            self.get_parameter('hsv_v_max').value,
        ])

        mask = cv2.inRange(hsv, lower, upper)

        # Morphology — usuwa szum i łączy fragmenty rurociągu
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        edges = cv2.Canny(mask, 50, 150)

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.get_parameter('hough_threshold').value,
            minLineLength=self.get_parameter('hough_min_line_length').value,
            maxLineGap=self.get_parameter('hough_max_line_gap').value,
        )

        if lines is None:
            return msg

        # Znajdź dominującą linię — najdłuższą
        best_line = None
        best_length = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = np.hypot(x2 - x1, y2 - y1)
            if length > best_length:
                best_length = length
                best_line = line[0]

        if best_line is None:
            return msg

        x1, y1, x2, y2 = best_line
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0

        # offset_x: [-1.0, 1.0], 0 = centrum obrazu
        msg.offset_x = (cx - self.width / 2.0) / (self.width / 2.0)

        # kąt względem osi pionowej (Y)
        dx = x2 - x1
        dy = y2 - y1
        angle_rad = np.arctan2(dx, dy)
        msg.angle_deg = float(np.degrees(angle_rad))

        # confidence: stosunek długości dominującej linii do przekątnej obrazu
        diagonal = np.hypot(self.width, self.height)
        msg.confidence = float(min(best_length / diagonal, 1.0))

        # przybliżona szerokość rurociągu przez kontur maski
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            _, _, w, _ = cv2.boundingRect(largest)
            msg.line_width_px = int(w)

        msg.detected = True
        return msg

    def destroy_node(self):
        self.cam.stop()
        self.cam.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PipelineDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
