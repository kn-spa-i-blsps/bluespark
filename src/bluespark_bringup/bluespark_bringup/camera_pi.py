"""
camera_pi.py — shared launch logic for the camera Raspberry Pis (front, side, down).

Every camera Pi runs the same thing:
  1. (tcp mode only) rpicam-vid — MJPEG TCP stream server, respawned if it exits
  2. camera_node — reads the camera, publishes <ns>/camera/image_raw
  3. the role's detectors (YOLO, pipeline, ...), under the same namespace
All start at once: the detectors just subscribe camera/image_raw and work as
soon as frames arrive.

Only the namespace, frame_id and detector list differ, so each <role>.launch.py
is just:

    def generate_launch_description():
        return camera_pi_launch(
            namespace="front",
            frame_id="camera_front",
            detectors=[Node(package="bluespark_vision", executable="vision_node",
                            name="vision_node", output="screen")],
        )

All launch arguments stay overridable from the command line, e.g.:
    ros2 launch bluespark_bringup front.launch.py camera_mode:=usb fps:=15
    ros2 launch bluespark_bringup front.launch.py --show-args

If camera_node gets no frame for CAMERA_TIMEOUT (at startup or later), it exits
with code 1 and the launch starts it again (respawn), so a dead camera is
retried instead of silently producing nothing. The detectors keep running.
"""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

# camera_node: longest time without a frame before it gives up (seconds). Covers
# rpicam-vid startup, the TCP connect retries and the 2 s Picamera2 warm-up.
CAMERA_TIMEOUT = 20.0
# Delay before a camera_node that gave up is started again (seconds).
RESPAWN_DELAY = 2.0


def camera_pi_launch(
    namespace: str, frame_id: str, detectors: List[Node]
) -> LaunchDescription:
    """Camera stream + camera_node + `detectors`, all under `namespace`.

    `detectors` use relative topics (camera/image_raw, detected_objects, ...),
    which resolve under the namespace. They start together with the camera.
    """
    args = [
        DeclareLaunchArgument(
            "namespace",
            default_value=namespace,
            description="ROS namespace grouping this camera and its detectors.",
        ),
        DeclareLaunchArgument(
            "camera_mode",
            default_value="tcp",
            description="Camera backend: tcp (rpicam-vid stream, Pi) | usb (laptop) "
            "| rpi (Picamera2 directly) | auto.",
        ),
        DeclareLaunchArgument(
            "camera_width", default_value="640", description="Capture width in pixels."
        ),
        DeclareLaunchArgument(
            "camera_height", default_value="480", description="Capture height in pixels."
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value=frame_id,
            description="frame_id stamped on published images (identifies the camera).",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="30.0",
            description="Capture (rpicam-vid) and publish rate, frames per second.",
        ),
        DeclareLaunchArgument(
            "stream_port",
            default_value="5000",
            description="TCP port of the rpicam-vid stream (tcp mode).",
        ),
    ]

    ns = LaunchConfiguration("namespace")
    camera_mode = LaunchConfiguration("camera_mode")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    fps = LaunchConfiguration("fps")
    stream_port = LaunchConfiguration("stream_port")

    # --- 1. TCP camera stream (Pi only): plain OS process, outside the namespace.
    # rpicam-vid --listen serves a single client and exits when it disconnects,
    # so respawn it; camera_node's capture thread then reconnects.
    camera_stream = ExecuteProcess(
        name="rpicam_vid",
        condition=IfCondition(PythonExpression(["'", camera_mode, "' == 'tcp'"])),
        cmd=[
            "rpicam-vid",
            "-t", "0",
            "--width", camera_width,
            "--height", camera_height,
            "--framerate", fps,
            "--codec", "mjpeg",
            "--inline",
            "--listen",
            "-o", ["tcp://0.0.0.0:", stream_port],
        ],
        respawn=True,
        respawn_delay=1.0,
        output="screen",
    )

    # --- 2. camera_node. Launch arguments are strings that launch_ros parses as
    # YAML ("30" -> int, "30.0" -> float); camera_node's parameters are strictly
    # typed, so pin each type here. Then fps:=30 or frame_id:=1 work instead of
    # crashing the node with InvalidParameterTypeException.
    camera = GroupAction(
        [
            PushRosNamespace(ns),
            Node(
                package="bluespark_vision",
                executable="camera_node",
                name="camera_node",
                parameters=[
                    {
                        "camera_mode": ParameterValue(camera_mode, value_type=str),
                        "camera_width": ParameterValue(camera_width, value_type=int),
                        "camera_height": ParameterValue(camera_height, value_type=int),
                        "frame_id": ParameterValue(
                            LaunchConfiguration("frame_id"), value_type=str
                        ),
                        "fps": ParameterValue(fps, value_type=float),
                        "stream_port": ParameterValue(stream_port, value_type=int),
                        "frame_timeout": CAMERA_TIMEOUT,
                    }
                ],
                output="screen",
                # Gives up after CAMERA_TIMEOUT without frames -> started again.
                respawn=True,
                respawn_delay=RESPAWN_DELAY,
            ),
        ]
    )

    # --- 3. detectors, same namespace, no waiting.
    detector_group = GroupAction([PushRosNamespace(ns), *detectors])

    return LaunchDescription([*args, camera_stream, camera, detector_group])
