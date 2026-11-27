import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import Image

from .camera import UniversalCamera
from .image_utils import cv2_to_ros_image


class UniversalCameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_node")

        # --- Parameters (set per camera from the launch file) ---
        # camera_mode : "auto" | "usb" | "rpi" | "tcp"  — which capture backend
        # camera_width / camera_height : capture resolution
        # frame_id : identifies which physical camera the frame came from
        #            (propagates through the detectors to the behaviour tree)
        # fps : publish rate; timer period is 1/fps
        # stream_port : TCP port of the rpicam-vid stream (tcp mode only)
        # frame_timeout : no frame for this long (s) -> the node exits with code 1
        #                 and the launch starts it again (respawn) (0 = never)

        # declare_parameter() returns the Parameter; the typed getters give a
        # plain str / int / float (Parameter.value is typed Any | None).
        def param(name, default):
            return self.declare_parameter(name, default).get_parameter_value()

        camera_mode = param("camera_mode", "auto").string_value
        camera_width = param("camera_width", 640).integer_value
        camera_height = param("camera_height", 480).integer_value
        self.frame_id = param("frame_id", "camera_link").string_value
        fps = param("fps", 30.0).double_value
        stream_port = param("stream_port", 5000).integer_value
        self.frame_timeout = param("frame_timeout", 20.0).double_value
        # Counts from startup, so it also covers "the camera never came up".
        self.last_frame_time = time.monotonic()
        # Done = the watchdog gave up; main() then exits with code 1.
        self.gave_up = Future()

        self.camera = UniversalCamera(
            width=camera_width,
            height=camera_height,
            mode=camera_mode,
            tcp_port=stream_port,
        )

        # Relative topic name: under a launch namespace like "down" this
        # resolves to /down/camera/image_raw automatically.
        self.publisher = self.create_publisher(Image, "camera/image_raw", 10)

        timer_period = 1.0 / fps  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f"Publishing camera feed (mode={camera_mode}, "
            f"{camera_width}x{camera_height}@{fps}fps, frame_id={self.frame_id})"
        )

    def timer_callback(self):
        ret, frame = self.camera.read()
        if not ret or frame is None:
            # do not spam logs if TCP is emtpy
            silent = time.monotonic() - self.last_frame_time
            if self.frame_timeout > 0 and silent > self.frame_timeout and not self.gave_up.done():
                self.get_logger().error(
                    f"No camera frame for {self.frame_timeout:g} s, exiting (code 1)")
                self.gave_up.set_result(True)
            return
        self.last_frame_time = time.monotonic()

        msg = cv2_to_ros_image(
            frame,
            frame_id=self.frame_id,
            stamp=self.get_clock().now().to_msg(),
        )
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UniversalCameraPublisher()
    try:
        # Runs until the watchdog gives up or rclpy shuts down (Ctrl+C).
        rclpy.spin_until_future_complete(node, node.gave_up)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.camera.release()
        node.destroy_node()
        rclpy.try_shutdown()
    if node.gave_up.done():
        sys.exit(1)


if __name__ == "__main__":
    main()