"""
governer.launch.py — main / "governer" Raspberry Pi.

This Pi has NO camera. It runs the flight-control stack and (optionally) the
behaviour tree:

  mavros            -> talks to the Pixhawk (serial)
  rc_override_node  -> sends PWM to the thrusters (control/set_* services)
  autonomy (py_trees) -> the mission behaviour tree   [only if run_autonomy:=true]

Everything starts AT ONCE, the launch does not wait for anything. Each node
copes with its dependencies itself (ROS 2 style):
  * rc_override_node just starts; its overrides reach the Pixhawk as soon as
    mavros subscribes. Arming / modes go straight to mavros (no middleman).
  * the mission (bluespark_autonomy.mission_runtime) waits up to setup_timeout
    for the FCU link (/mavros/state connected) and for every service it uses;
    missing after that -> it exits with code 2 before doing anything.
  * rc_override_node is respawned if it dies, so a crash there does not take
    the whole stack down.
Check the stack from outside with:  vehicle ready   (./bs start does that).

Launch return code (read by run_governer.sh):
  * mission exits 0                -> launch shuts down, rc 0 = "mission complete"
  * mission exits != 0 (failed,    -> launch shuts down, rc 1 = crash
    setup failed, crashed)            (run_governer.sh disarms + restarts)
  * Ctrl+C / SIGINT                -> normal shutdown, not turned into rc 1
  (bluespark_bringup/launch_exit.py)

DEV vs PROD:
  * PROD (run_governer.sh): launches with defaults -> autonomy runs.
  * DEV / bench testing: launch with run_autonomy:=false to bring up ONLY the
    stack (mavros + control), then run your mission by hand:
        ros2 launch bluespark_bringup governer.launch.py run_autonomy:=false
        # then, in another shell in the same container:
        ros2 run bluespark_autonomy benchTest --ros-args -p mission:=wiggle
    This is what docker-compose.bench.yml uses so you never hand-start mavros.
SCOPE — this launch only STARTS the stack and turns the mission's exit code into
its own. It does NOT restart the stack or disarm. Those stay in run_governer.sh.

Arming / mode setting is intentionally NOT done here — the mission (py_trees)
or the supervising script decides when to arm, so the vehicle is never armed
merely because the stack came up.
"""

from bluespark_bringup.launch_exit import end_launch_on_exit
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# How long the mission waits for the FCU link + its services (seconds). Covers
# mavros start + Pixhawk connect on the Pi (~30 s worst case seen so far).
SETUP_TIMEOUT = 45.0
# Delay before a dead rc_override_node is started again (seconds).
RESPAWN_DELAY = 2.0


def _timeline(context):
    """Session timeline recorder (./bs), only when log_dir is given."""
    log_dir = LaunchConfiguration("log_dir").perform(context).strip()
    if not log_dir:
        return []
    return [
        Node(
            package="bluespark_bringup",
            executable="timeline_recorder",
            parameters=[{"log_dir": log_dir}],
            output="screen",
        )
    ]


def generate_launch_description():
    # --- Launch arguments ---
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url",
        default_value="serial:///dev/ttyACM0:115200",
        description="MAVLink connection to the flight controller (Pixhawk).",
    )
    autonomy_exe_arg = DeclareLaunchArgument(
        "autonomy_executable",
        default_value="hardcoded_gate",
        description="Which py_trees entry point to run from bluespark_autonomy.",
    )
    run_autonomy_arg = DeclareLaunchArgument(
        "run_autonomy",
        default_value="true",
        description="If false, bring up only the stack (mavros + control) and "
        "do NOT start any mission. Use for bench testing, then run "
        "the mission by hand with ros2 run.",
    )

    log_dir_arg = DeclareLaunchArgument(
        "log_dir",
        default_value="",
        description="Session folder (./bs, run_governer.sh): if set, timeline_recorder writes "
        "<log_dir>/timeline.log. Empty (default) = no recorder, nothing changes.",
    )

    fcu_url = LaunchConfiguration("fcu_url")
    autonomy_executable = LaunchConfiguration("autonomy_executable")
    run_autonomy = LaunchConfiguration("run_autonomy")

    # --- mavros ---
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
        ),
        launch_arguments={"fcu_url": fcu_url}.items(),
    )

    # --- control node: no dependency at startup, respawned if it dies ---
    rc_override = Node(
        package="bluespark_control",
        executable="rc_override_node",
        name="rc_override_node",
        output="screen",
        respawn=True,
        respawn_delay=RESPAWN_DELAY,
    )

    # --- autonomy / py_trees --- only when run_autonomy is true. Waits for the
    # FCU and its services by itself (setup_timeout); its exit ends the launch.
    autonomy = Node(
        package="bluespark_autonomy",
        executable=autonomy_executable,
        name="autonomy_node",
        output="screen",
        parameters=[{"setup_timeout": SETUP_TIMEOUT}],
        condition=IfCondition(run_autonomy),
    )
    # Never fires when autonomy is not started (run_autonomy:=false).
    on_mission_exit = RegisterEventHandler(
        OnProcessExit(target_action=autonomy, on_exit=end_launch_on_exit))

    return LaunchDescription(
        [
            fcu_url_arg,
            autonomy_exe_arg,
            run_autonomy_arg,
            log_dir_arg,
            OpaqueFunction(function=_timeline),  # first, so it sees mavros start
            mavros,
            rc_override,
            autonomy,
            on_mission_exit,
        ]
    )
