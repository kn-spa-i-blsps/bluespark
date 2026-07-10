"""
front.launch.py — front-camera Raspberry Pi.

Brings up, under the "front" namespace:
  - (tcp mode only) rpicam-vid  — MJPEG TCP stream server on :5000
  - camera_node   (owns the front camera, publishes front/camera/image_raw)
  - vision_node   (YOLO detection + 3D pose, publishes front/detected_objects)

The front camera looks ahead (gates, wall images, targets), so it runs YOLO
rather than the bottom-facing classic detectors. The behaviour tree reads
/front/detected_objects.

Camera source:
  - tcp  (default, on the Pi): rpicam-vid streams the CSI camera over TCP.
  - usb  (laptop dev): camera_node reads a USB camera; no stream is started.

    ros2 launch bluespark_bringup front.launch.py camera_mode:=usb
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    # --- Launch arguments ---
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='front',
        description='ROS namespace grouping this camera and its detector.')
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
        'frame_id', default_value='camera_front',
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

    # Only run the rpicam-vid TCP stream when camera_mode == 'tcp'.
    is_tcp = IfCondition(PythonExpression(["'", camera_mode, "' == 'tcp'"]))

    # --- TCP camera stream (Pi only), plain OS process, outside the namespace ---
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

    # --- Camera + YOLO detector, under the namespace ---
    group = GroupAction([
        PushRosNamespace(namespace),

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

        # YOLO: subscribes to relative camera/image_raw -> /front/camera/image_raw,
        # publishes relative detected_objects -> /front/detected_objects.
        Node(
            package='bluespark_vision',
            executable='vision_node',
            name='vision_node',
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
        camera_stream,
        group,
    ])
