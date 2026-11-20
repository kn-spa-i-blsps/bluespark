"""
governer.launch.py — main / "governer" Raspberry Pi.

This Pi has NO camera. It runs the flight-control stack and (optionally) the
behaviour tree:

  mavros            -> talks to the Pixhawk (serial)
  rc_override_node  -> sends PWM to the thrusters (control/set_* services)
  vehicle_manager   -> arming + mode services (/manager/set_arming, /manager/set_mode)
  autonomy (py_trees) -> the mission behaviour tree   [only if run_autonomy:=true]

Startup ORDER matters, so each step waits until the previous one is really
ready (WaitForMessage, see bluespark_bringup/wait_for.py), not a fixed delay:
  1. mavros
  2. wait for /mavros/state with connected == True        (Pixhawk link is up)
  3. rc_override_node + vehicle_manager_node, together
  4. wait until both published their name on /bluespark/ready  (services created)
  5. autonomy                                                   [optional]
If a wait times out (MAVROS_TIMEOUT / STACK_TIMEOUT), the launch shuts down with
return code 1, so run_governer.sh treats it as a crash (disarm + restart).
Debug readiness with:
    ros2 topic echo /bluespark/ready --qos-durability transient_local

DEV vs PROD:
  * PROD (run_governer.sh): launches with defaults -> autonomy runs.
  * DEV / bench testing: launch with run_autonomy:=false to bring up ONLY the
    stack (mavros + control), then run your mission by hand:
        ros2 launch bluespark_bringup governer.launch.py run_autonomy:=false
        # then, in another shell in the same container:
        ros2 run bluespark_autonomy benchTest --ros-args -p mission:=wiggle
    This is what docker-compose.bench.yml uses so you never hand-start mavros.
SCOPE — this launch only STARTS the stack in order. It does NOT handle restart
on crash, clean-exit vs crash, or disarm-on-exit. Those stay in run_governer.sh.

Arming / mode setting is intentionally NOT done here — the mission (py_trees)
or the supervising script decides when to arm, so the vehicle is never armed
merely because the stack came up.
"""

from bluespark_bringup.wait_for import READY_QOS, READY_TOPIC, WaitForMessage, ready_all
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from mavros_msgs.msg import State
from std_msgs.msg import String

# How long to wait for each step before giving up (seconds).
MAVROS_TIMEOUT = 30.0  # mavros started -> FCU connected
STACK_TIMEOUT = 15.0  # control nodes started -> both announced ready


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

    fcu_url = LaunchConfiguration("fcu_url")
    autonomy_executable = LaunchConfiguration("autonomy_executable")
    run_autonomy = LaunchConfiguration("run_autonomy")

    # --- 1. mavros ---
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
        ),
        launch_arguments={"fcu_url": fcu_url}.items(),
    )

    # --- 3. control nodes (started together once mavros is connected) ---
    rc_override = Node(
        package="bluespark_control",
        executable="rc_override_node",
        name="rc_override_node",
        output="screen",
    )
    vehicle_manager = Node(
        package="bluespark_control",
        executable="vehicle_manager_node",
        name="vehicle_manager_node",
        output="screen",
    )

    # --- 5. autonomy / py_trees --- only when run_autonomy is true.
    autonomy = Node(
        package="bluespark_autonomy",
        executable=autonomy_executable,
        name="autonomy_node",
        output="screen",
        condition=IfCondition(run_autonomy),
    )

    # --- 4. wait for both control nodes, then start autonomy ---
    wait_for_control = WaitForMessage(
        READY_TOPIC,
        String,
        ready_all({"rc_override", "manager"}),
        qos=READY_QOS,
        timeout=STACK_TIMEOUT,
        then=[autonomy],
    )

    # --- 2. wait for the FCU link, then start the control nodes ---
    wait_for_mavros = WaitForMessage(
        "/mavros/state",
        State,
        lambda msg: msg.connected,
        description="connected",
        timeout=MAVROS_TIMEOUT,
        then=[rc_override, vehicle_manager, wait_for_control],
    )

    return LaunchDescription(
        [
            fcu_url_arg,
            autonomy_exe_arg,
            run_autonomy_arg,
            mavros,
            wait_for_mavros,
        ]
    )
