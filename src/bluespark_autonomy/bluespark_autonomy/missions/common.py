"""
common.py — building blocks shared by the test missions.

Every test mission:
  1. sets the Pixhawk's own depth hold: mode ALT_HOLD, and CONFIRMS it
     (fails if the Pixhawk does not switch, e.g. no depth sensor),
  2. arms (ArmRobot fails if the Pixhawk refuses, e.g. a pre-arm check),
  3. moves open-loop (fixed PWM for a fixed time, MoveRC),
  4. stops, disarms.
Any step that fails stops the mission there (exit code 3); ./bs mission then
disarms and sets all thrusters to 1500 anyway.

Movements are time-based: the durations / PWMs are ROS parameters, tune them
at the pool without changing code, e.g.
    ./bs mission forwardTurnBack --ros-args -p turn_sec:=3.5
Which way a PWM above 1500 turns / moves the vehicle is not confirmed yet:
if it goes the wrong way, use the other side of 1500 (e.g. 1400 instead of 1600).
"""

from rcl_interfaces.msg import ParameterDescriptor

from bluespark_autonomy.behaviours.control import ArmRobot, SetFlightMode, WaitForMode
from bluespark_autonomy.behaviours.hardcoded_actions import MoveRC


def params(node, defaults):
    """Declare the mission's parameters, log the values used, return them as a dict.

    Typing is loose on purpose: `-p turn_sec:=3` (an int) is fine for a float
    default and is converted to 3.0; a value that is not a number fails the
    mission with a clear message.
    """
    loose = ParameterDescriptor(dynamic_typing=True)
    values = {}
    for key, default in defaults.items():
        raw = node.declare_parameter(key, default, loose).value
        try:
            values[key] = type(default)(raw)
        except (TypeError, ValueError):
            raise ValueError(f"parameter {key}:={raw!r} must be a number like {default!r}")
    node.get_logger().info(
        "Parameters: " + ", ".join(f"{key}={value}" for key, value in values.items())
    )
    return values


def start_in_althold(mission):
    """ALT_HOLD (Pixhawk depth hold), confirmed, then arm."""
    mission.add_child(SetFlightMode(name="Set ALT_HOLD", mode="ALT_HOLD"))
    mission.add_child(WaitForMode(name="Confirm ALT_HOLD", mode="ALT_HOLD", timeout_sec=5.0))
    mission.add_child(ArmRobot(name="Arm", arm=True))


def pause(name, seconds):
    """All thrusters 1500 for a while (let the vehicle settle)."""
    return MoveRC(name, duration_sec=seconds)


def finish(mission, settle_sec=1.0):
    mission.add_child(pause("Stop", settle_sec))
    mission.add_child(ArmRobot(name="Disarm", arm=False))
