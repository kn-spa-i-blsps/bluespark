"""
down.launch.py — bottom-camera Raspberry Pi.

Brings up, under the "down" namespace (shared logic: bluespark_bringup/camera_pi.py):
  - (tcp mode only) rpicam-vid  — MJPEG TCP stream server on :5000
  - camera_node        (owns the bottom camera, publishes down/camera/image_raw)
  and, once the first camera frame arrives:
  - pipeline_node      (follows the pipeline)
  - path_marker_node   (finds the path marker)
  - bin_detector_node  (finds the bin + its symbol)

Everything ROS is grouped under the 'down' namespace, so the relative topics
in the nodes (camera/image_raw, pipeline/data, ...) resolve to
/down/camera/image_raw, /down/pipeline/data, etc. The behaviour tree reads
those /down/... topics.

Camera source:
  - tcp  (default, on the Pi): rpicam-vid streams the CSI camera over TCP and
    camera_node reads it. This launch starts the stream for you.
  - usb  (on a laptop, for dev): camera_node reads a USB camera directly; the
    rpicam-vid stream is NOT started.

    ros2 launch bluespark_bringup down.launch.py camera_mode:=usb
    ros2 launch bluespark_bringup down.launch.py --show-args   # all options
"""

from bluespark_bringup.camera_pi import camera_pi_launch
from launch_ros.actions import Node


def generate_launch_description():
    return camera_pi_launch(
        namespace="down",
        frame_id="camera_down",
        detectors=[
            # Each subscribes to the relative camera/image_raw -> /down/camera/image_raw.
            Node(
                package="bluespark_vision",
                executable="pipeline_node",
                name="pipeline_node",
                output="screen",
                # HSV / Hough thresholds use the node defaults for now.
                # To tune them without rebuilding, drop in a params file later:
                # parameters=[PathJoinSubstitution([
                #     FindPackageShare('bluespark_bringup'), 'params', 'down.yaml'])],
            ),
            Node(
                package="bluespark_vision",
                executable="path_marker_node",
                name="path_marker_node",
                output="screen",
            ),
            Node(
                package="bluespark_vision",
                executable="bin_detector_node",
                name="bin_detector_node",
                output="screen",
            ),
        ],
    )
