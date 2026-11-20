"""
front.launch.py — front-camera Raspberry Pi.

Brings up, under the "front" namespace (shared logic: bluespark_bringup/camera_pi.py):
  - (tcp mode only) rpicam-vid  — MJPEG TCP stream server on :5000
  - camera_node   (owns the front camera, publishes front/camera/image_raw)
  - vision_node   (YOLO detection + 3D pose, publishes front/detected_objects),
                  started once the first camera frame arrives

The front camera looks ahead (gates, wall images, targets), so it runs YOLO
rather than the bottom-facing classic detectors. The behaviour tree reads
/front/detected_objects.

Camera source:
  - tcp  (default, on the Pi): rpicam-vid streams the CSI camera over TCP.
  - usb  (laptop dev): camera_node reads a USB camera; no stream is started.

    ros2 launch bluespark_bringup front.launch.py camera_mode:=usb
    ros2 launch bluespark_bringup front.launch.py --show-args   # all options
"""

from bluespark_bringup.camera_pi import camera_pi_launch
from launch_ros.actions import Node


def generate_launch_description():
    return camera_pi_launch(
        namespace="front",
        frame_id="camera_front",
        detectors=[
            # YOLO: relative camera/image_raw -> /front/camera/image_raw,
            # relative detected_objects -> /front/detected_objects.
            Node(
                package="bluespark_vision",
                executable="vision_node",
                name="vision_node",
                output="screen",
            ),
        ],
    )
