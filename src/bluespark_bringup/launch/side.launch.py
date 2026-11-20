"""
side.launch.py — side-camera Raspberry Pi.

Brings up, under the "side" namespace (shared logic: bluespark_bringup/camera_pi.py):
  - (tcp mode only) rpicam-vid  — MJPEG TCP stream server on :5000
  - camera_node   (owns the side camera, publishes side/camera/image_raw)
  - vision_node   (YOLO detection + 3D pose, publishes side/detected_objects),
                  started once the first camera frame arrives

The side camera looks sideways (side tasks, images on side walls), so it runs YOLO
rather than the bottom-facing classic detectors. The behaviour tree reads
/side/detected_objects.

Camera source:
  - tcp  (default, on the Pi): rpicam-vid streams the CSI camera over TCP.
  - usb  (laptop dev): camera_node reads a USB camera; no stream is started.

    ros2 launch bluespark_bringup side.launch.py camera_mode:=usb
    ros2 launch bluespark_bringup side.launch.py --show-args   # all options
"""

from bluespark_bringup.camera_pi import camera_pi_launch
from launch_ros.actions import Node


def generate_launch_description():
    return camera_pi_launch(
        namespace="side",
        frame_id="camera_side",
        detectors=[
            # YOLO: relative camera/image_raw -> /side/camera/image_raw,
            # relative detected_objects -> /side/detected_objects.
            Node(
                package="bluespark_vision",
                executable="vision_node",
                name="vision_node",
                output="screen",
            ),
        ],
    )
