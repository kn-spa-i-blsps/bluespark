"""
down.launch.py — bottom-camera Raspberry Pi.

Brings up, under the "down" namespace:
  - (tcp mode only) rpicam-vid  — MJPEG TCP stream server on :5000
  - camera_node        (owns the bottom camera, publishes down/camera/image_raw)
  - pipeline_node      (follows the pipeline)
  - path_marker_node   (finds the path marker)
  - bin_detector_node  (finds the bin + its symbol)

Everything ROS is grouped under PushRosNamespace('down'), so the relative topics
in the nodes (camera/image_raw, pipeline/data, ...) resolve to
/down/camera/image_raw, /down/pipeline/data, etc. The behaviour tree reads
those /down/... topics.

Camera source:
  - tcp  (default, on the Pi): rpicam-vid streams the CSI camera over TCP and
    camera_node reads it. This launch starts the stream for you.
  - usb  (on a laptop, for dev): camera_node reads a USB camera directly; the
    rpicam-vid stream is NOT started.

    ros2 launch bluespark_bringup down.launch.py camera_mode:=usb
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    # --- Launch arguments (the knobs that differ per camera) ---
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='down',
        description='ROS namespace grouping this camera and its detectors.')
    camera_mode_arg = DeclareLaunchArgument(
        'camera_mode', default_value='tcp',
        description='Camera backend: auto | usb | rpi | tcp.')
    camera_width_arg = DeclareLaunchArgument(
        'camera_width', default_value='640',
        description='Capture width in pixels.')
    camera_height_arg = DeclareLaunchArgument(
        'camera_height', default_value='480',
        description='Capture height in pixels.')
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='camera_down',
        description='frame_id stamped on published images (identifies the camera).')
    fps_arg = DeclareLaunchArgument(
        'fps', default_value='30.0',
        description='Publish rate for camera frames.')

    namespace = LaunchConfiguration('namespace')
    camera_mode = LaunchConfiguration('camera_mode')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    frame_id = LaunchConfiguration('frame_id')
    fps = LaunchConfiguration('fps')

    # Condition: only run the rpicam-vid TCP stream when camera_mode == 'tcp'.
    is_tcp = IfCondition(PythonExpression(["'", camera_mode, "' == 'tcp'"]))

    # --- TCP camera stream (Pi only) ---
    # This is a plain OS process, NOT a ROS node, so it lives outside the
    # namespace group. camera_node connects to it as a TCP client; camera.py
    # already reconnects if the stream isn't up yet, so no strict ordering is
    # needed. Width/height are passed from the launch args so the stream matches
    # what camera_node expects.
    camera_stream = ExecuteProcess(
        condition=is_tcp,
        cmd=[
            'rpicam-vid',
            '-t', '0',
            '--width', camera_width,
            '--height', camera_height,
            '--framerate', '30',
            '--codec', 'mjpeg',
            '--inline',
            '--listen',
            '-o', 'tcp://0.0.0.0:5000',
        ],
        output='screen',
    )

    # --- The camera + detector group, all under the same namespace ---
    group = GroupAction([
        PushRosNamespace(namespace),

        # Camera: the single frame source for this Pi.
        Node(
            package='bluespark_vision',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'camera_mode': camera_mode,
                'camera_width': camera_width,
                'camera_height': camera_height,
                'frame_id': frame_id,
                'fps': fps,
            }],
            output='screen',
        ),

        # Detectors: each subscribes to the relative camera/image_raw, which
        # under this namespace resolves to /down/camera/image_raw.
        Node(
            package='bluespark_vision',
            executable='pipeline_node',
            name='pipeline_node',
            output='screen',
            # HSV / Hough thresholds use the node defaults for now.
            # To tune them without rebuilding, drop in a params file later:
            # parameters=[PathJoinSubstitution([
            #     FindPackageShare('bluespark_bringup'), 'params', 'down.yaml'])],
        ),
        Node(
            package='bluespark_vision',
            executable='path_marker_node',
            name='path_marker_node',
            output='screen',
        ),
        Node(
            package='bluespark_vision',
            executable='bin_detector_node',
            name='bin_detector_node',
            output='screen',
        ),
    ])

    return LaunchDescription([
        namespace_arg,
        camera_mode_arg,
        camera_width_arg,
        camera_height_arg,
        frame_id_arg,
        fps_arg,
        camera_stream,   # starts only in tcp mode
        group,
    ])