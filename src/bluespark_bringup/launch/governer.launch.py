"""
governer.launch.py — main / "governer" Raspberry Pi.

This Pi has NO camera. It runs the flight-control stack and (optionally) the
behaviour tree:

  mavros            -> talks to the Pixhawk (serial)
  rc_override_node  -> sends PWM to the thrusters (control/set_* services)
  vehicle_manager   -> arming + mode services (/manager/set_arming, /manager/set_mode)
  autonomy (py_trees) -> the mission behaviour tree   [only if run_autonomy:=true]

Startup ORDER matters (each depends on the previous), so nodes are staggered
with TimerAction delays that mirror the sleeps in the old test03.sh:
  mavros            at t=0
  rc_override       at t=3   (after mavros link)
  vehicle_manager   at t=5
  autonomy          at t=12  (after MAVROS is connected)   [optional]

DEV vs PROD:
  * PROD (run_governer.sh): launches with defaults -> autonomy runs.
  * DEV / bench testing: launch with run_autonomy:=false to bring up ONLY the
    stack (mavros + control), then run your mission by hand:
        ros2 launch bluespark_bringup governer.launch.py run_autonomy:=false
        # then, in another shell in the same container:
        ros2 run bluespark_autonomy benchTest --ros-args -p mission:=wiggle
    This is what docker-compose.dev.yml uses so you never hand-start mavros.

SCOPE — this launch only STARTS the stack in order. It does NOT handle restart
on crash, clean-exit vs crash, or disarm-on-exit. Those stay in run_governer.sh.

Arming / mode setting is intentionally NOT done here — the mission (py_trees)
or the supervising script decides when to arm, so the vehicle is never armed
merely because the stack came up.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # --- Launch arguments ---
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='serial:///dev/ttyACM0:115200',
        description='MAVLink connection to the flight controller (Pixhawk).')
    autonomy_exe_arg = DeclareLaunchArgument(
        'autonomy_executable', default_value='hardcoded_gate',
        description='Which py_trees entry point to run from bluespark_autonomy.')
    run_autonomy_arg = DeclareLaunchArgument(
        'run_autonomy', default_value='true',
        description='If false, bring up only the stack (mavros + control) and '
                    'do NOT start any mission. Use for bench testing, then run '
                    'the mission by hand with ros2 run.')

    fcu_url = LaunchConfiguration('fcu_url')
    autonomy_executable = LaunchConfiguration('autonomy_executable')
    run_autonomy = LaunchConfiguration('run_autonomy')

    # --- mavros (t=0) ---
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'apm.launch'])
        ),
        launch_arguments={'fcu_url': fcu_url}.items(),
    )

    # --- rc_override_node (t=3) ---
    rc_override = TimerAction(
        period=3.0,
        actions=[Node(
            package='bluespark_control',
            executable='rc_override_node',
            name='rc_override_node',
            output='screen',
        )],
    )

    # --- vehicle_manager_node (t=5) ---
    vehicle_manager = TimerAction(
        period=5.0,
        actions=[Node(
            package='bluespark_control',
            executable='vehicle_manager_node',
            name='vehicle_manager_node',
            output='screen',
        )],
    )

    # --- autonomy / py_trees (t=12) --- only when run_autonomy is true.
    autonomy = TimerAction(
        period=12.0,
        actions=[Node(
            package='bluespark_autonomy',
            executable=autonomy_executable,
            name='autonomy_node',
            output='screen',
            condition=IfCondition(run_autonomy),
        )],
    )

    return LaunchDescription([
        fcu_url_arg,
        autonomy_exe_arg,
        run_autonomy_arg,
        mavros,
        rc_override,
        vehicle_manager,
        autonomy,
    ])