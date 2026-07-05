import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .camera import UniversalCamera

class UniversalCameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_node")
        
        self.camera = UniversalCamera(mode="tcp") 
        
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        
        timer_period = 0.033 # ~30 fps in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Start publishing camera feed on /camera/image_raw")

    def timer_callback(self):
        ret, frame = self.camera.read()
        if not ret or frame is None:
            # do not spam logs if TCP is emtpy
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
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