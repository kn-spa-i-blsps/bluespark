import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import os

from bluespark_interfaces.msg import DetectedObject
from bluespark_interfaces.msg import DetectedObjectArray
from .detector import ObjectDetector
from .simple_distance_calculator import SimpleDistanceCalculator
from .camera import UniversalCamera


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node_publisher")
        self.publisher = self.create_publisher(
            DetectedObjectArray, "detected_objects", 10
        )

        pkg_share_dir = get_package_share_directory("bluespark_vision")
        model_name = "jedy8.pt"
        model_path = os.path.join(pkg_share_dir, "ml_models", model_name)
        self.detector = ObjectDetector(str(model_path))
        self.distance_calc = SimpleDistanceCalculator()
        self.camera = UniversalCamera(mode="usb")  # usb/rpi/auto

        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.camera.read()
        if not ret or frame is None:
            self.get_logger().warning("No camera frame, skipping iteration.")
            return

        array_msg = DetectedObjectArray()
        array_msg.header.stamp = self.get_clock().now().to_msg()
        array_msg.header.frame_id = "camera_link"

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
        print("[INFO] Releasing the camera and closing the node.")
        if hasattr(node, "camera") and node.camera is not None:
            node.camera.release()

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

