import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import os

from bluespark_interfaces.msg import DetectedObject
from bluespark_interfaces.msg import DetectedObjectArray
from .detector import ObjectDetector
from .simple_distance_calculator import SimpleDistanceCalculator
from .image_utils import ros_image_to_cv2

from rclpy.qos import QoSProfile, HistoryPolicy
from sensor_msgs.msg import Image

class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node_publisher")
        self.publisher = self.create_publisher(
            DetectedObjectArray, "detected_objects", 10
        )

        pkg_share_dir = get_package_share_directory("bluespark_vision")
        model_name = "yolo11n.pt"
        model_path = os.path.join(pkg_share_dir, "ml_models", model_name)
        self.detector = ObjectDetector(str(model_path))
        self.distance_calc = SimpleDistanceCalculator()

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            qos_profile
        )

    def image_callback(self, msg: Image):
        try:
            frame = ros_image_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        array_msg = DetectedObjectArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = msg.header.frame_id

        detections = self.detector.detect_objects(frame, threshold=0.5, imgsz=224)
        for detection in detections:
            x1, y1, x2, y2, label, conf = detection
            bbox = (x1, y1, x2, y2)

            pose_info = self.distance_calc.calculate_pose(bbox, label)

            if pose_info is None:
                continue

            obj_msg = DetectedObject()
            obj_msg.label = label
            obj_msg.confidence = float(conf)

            obj_msg.pos_x, obj_msg.pos_y, obj_msg.pos_z = [
                float(v) for v in pose_info["pos"]
            ]
            obj_msg.cam_h_angle_deg, obj_msg.cam_v_angle_deg = [
                float(v) for v in pose_info["cam_angles"]
            ]

            obj_msg.obj_rotation_deg = float(pose_info["obj_rotation"])

            obj_msg.x_center = int(pose_info["center_px"][0])
            obj_msg.y_center = int(pose_info["center_px"][1])

            array_msg.objects.append(obj_msg)

        self.publisher.publish(array_msg)


def main(args=None):
    rclpy.init(args=args)

    node = VisionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Closing signal received (Ctrl+C).")
    finally:
        print("[INFO] closing the node.")

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

