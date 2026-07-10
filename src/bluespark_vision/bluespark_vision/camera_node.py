import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from .image_utils import cv2_to_ros_image

from .camera import UniversalCamera


class UniversalCameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_node")

        # --- Parameters (set per camera from the launch file) ---
        # camera_mode : "auto" | "usb" | "rpi" | "tcp"  — which capture backend
        # camera_width / camera_height : capture resolution
        # frame_id : identifies which physical camera the frame came from
        #            (propagates through the detectors to the behaviour tree)
        # fps : publish rate; timer period is 1/fps

        self.declare_parameter("camera_mode", "auto")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("fps", 30.0)

        camera_mode = self.get_parameter("camera_mode").value
        camera_width = self.get_parameter("camera_width").value
        camera_height = self.get_parameter("camera_height").value
        self.frame_id = self.get_parameter("frame_id").value
        fps = self.get_parameter("fps").value

        self.camera = UniversalCamera(
            width=camera_width,
            height=camera_height,
            mode=camera_mode,
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
            return

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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, "camera") and node.camera is not None:
            node.camera.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()