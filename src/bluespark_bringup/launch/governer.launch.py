"""
brain.launch.py — main / "brain" Raspberry Pi.

This Pi has NO camera. It runs the flight-control stack and the behaviour tree:

  mavros            -> talks to the Pixhawk (serial)
  rc_override_node  -> sends PWM to the thrusters (control/set_* services)
  vehicle_manager   -> arming + mode services (/manager/set_arming, /manager/set_mode)
  autonomy (py_trees) -> the mission behaviour tree

Startup ORDER matters (each depends on the previous), so nodes are staggered
with TimerAction delays that mirror the sleeps in the old test03.sh:
  mavros            at t=0
  rc_override       at t=3   (after mavros link)
  vehicle_manager   at t=5
  autonomy          at t=12  (after MAVROS is connected)

SCOPE — this launch only STARTS the stack in order. It does NOT handle:
  - restart on crash
  - clean-exit vs crash distinction
  - disarm-on-exit safety
Those stay in the supervising bash script (run_brain.sh), which calls this
launch as a single process and watches it. Keeping the safety/lifecycle logic
in bash (where it is explicit) rather than in launch event handlers is a
deliberate choice.

Arming / mode setting is intentionally NOT done here — the mission (py_trees)
or the supervising script decides when to arm, so the vehicle is never armed
merely because the stack came up.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
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
        'autonomy_executable', default_value='approacherTest',
        description='Which py_trees entry point to run from bluespark_autonomy.')

    fcu_url = LaunchConfiguration('fcu_url')
    autonomy_executable = LaunchConfiguration('autonomy_executable')

    # --- mavros (t=0) ---
    # Included from the mavros package's apm.launch, same as test03.sh.
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

    # --- autonomy / py_trees (t=12), after MAVROS is connected ---
    autonomy = TimerAction(
        period=12.0,
        actions=[Node(
            package='bluespark_autonomy',
            executable=autonomy_executable,
            name='autonomy_node',
            output='screen',
        )],
    )

    return LaunchDescription([
        fcu_url_arg,
        autonomy_exe_arg,
        mavros,
        rc_override,
        vehicle_manager,
        autonomy,
    ])