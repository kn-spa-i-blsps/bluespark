"""
File: benchTest.py
Purpose:
    One entry point that runs one of several NAMED test missions, so you can
    validate the autonomy -> control -> RC path piece by piece and SEE the drone
    react. Pick the mission with a ROS param.

    Every mission ARMS at the start and DISARMS at the end (thrusters are on
    motors, we always want a real armed test), is guaranteed to FINISH ON ITS
    OWN (no mission hangs forever, so the trailing disarm always runs), and logs
    each step.

    SAFETY: arming makes props spin for real. Run with props physically safe.
    MoveRC.terminate() and the trailing ArmRobot(disarm) both send STOP; on
    Ctrl+C the tree shuts down and rc_override_node drops every channel to 1500.

RUN (inside the prod container on the governer, started manually):
    ros2 launch bluespark_bringup governer.launch.py autonomy_executable:=benchTest

    ros2 run bluespark_autonomy benchTest --ros-args -p mission:=wiggle
    ros2 run bluespark_autonomy benchTest --ros-args -p mission:=square -p duration:=2.0
    ros2 run bluespark_autonomy benchTest --ros-args -p mission:=approach -p approach_timeout:=20.0
    ros2 run bluespark_autonomy benchTest --ros-args \
        -p mission:=depth -p target_depth:=-0.5 -p depth_pid_mode:=p_only -p depth_timeout:=60.0

MISSIONS:
    wiggle    - yaw left, then yaw right. Smallest sanity check that RC reacts.
    surge/heave/yaw/sway/pitch/roll - one push on that axis, then stop.
    square    - surge/yaw/surge/yaw: proves a multi-step SEQUENCE runs in order.
    allaxes   - pulses every axis one after another: full wiring check.
    approach  - real ApproachGate wrapped in a hard Timeout so it always ends.
    depth     - real DepthControl behaviour: drive to target_depth and hold, so
                you can swim around and judge whether it makes sense. Wrapped in
                a Timeout so the test ends and disarms on its own.

PARAMS (all optional):
    mission (str)          default 'wiggle'
    pwm (int)              default 1600      single-axis PWM.
    duration (float)       default 2.0       seconds per movement step.
    approach_timeout (f)   default 20.0      hard cap (s) on approach.
    approach_label (str)   default 'person'  label ApproachGate chases.
    target_depth (float)   default -0.3      DESIRED depth, metres, NEGATIVE = below surface.
    depth_timeout (float)  default 60.0      hard cap (s) on the depth mission.
    depth_pid_mode (str)   default 'full'    'full' = the tuned-in-repo depth PID
                                             (kp200/ki10/kd20). 'p_only' = zero the
                                             I and D terms for this run (cleaner first
                                             read: reaches near target, small steady
                                             offset, no integrator wind-up / D noise).

NOTES:
  * DepthControl reads depth from the 'depth/current_depth' blackboard, which
    BlackboardManager fills straight from mavros/imu/static_pressure using a
    hardcoded atmospheric pressure (101325 Pa). On a high/low-pressure day this
    can offset the reading by up to ~0.2 m, so 'holding -0.3' may really be a bit
    off. Judge the BEHAVIOUR (does it drive the right way and settle?), and
    cross-check the absolute depth separately.
  * DepthControl does NOT gate on ALT_HOLD, so it acts in any mode. We still set
    ALT_HOLD first so the autopilot stabilises the other axes while you only
    command heave.
  * depth_pid_mode='p_only' mutates the depth PID gains on the shared
    AxisController instance created for THIS behaviour only; it does not edit
    pid.py. Nothing persists past this process.
"""

import signal

import rclpy
import py_trees
import py_trees_ros.trees
from py_trees.composites import Sequence
from py_trees.decorators import Timeout, FailureIsSuccess

from bluespark_autonomy.behaviours.hardcoded_actions import MoveRC
from bluespark_autonomy.behaviours.control import SetFlightMode, ArmRobot
from bluespark_autonomy.behaviours.actions import ApproachGate, AdjustDepth
from bluespark_autonomy.blackboard_manager import BlackboardManager


# ---------------------------------------------------------------------------
# Bodies. Each returns a LIST of children; arm/disarm added centrally.
# ---------------------------------------------------------------------------
def body_single(axis, pwm, duration):
    return [MoveRC(name=f"{axis} @ {pwm} ({duration}s)", duration_sec=duration, **{axis: pwm})]


def body_wiggle(duration):
    return [
        MoveRC(name="Yaw left", duration_sec=duration, yaw=1450),
        MoveRC(name="Center", duration_sec=1.0),
        MoveRC(name="Yaw right", duration_sec=duration, yaw=1550),
    ]


def body_square(duration):
    return [
        MoveRC(name="Forward 1", duration_sec=duration, surge=1600),
        MoveRC(name="Turn 1", duration_sec=duration, yaw=1600),
        MoveRC(name="Forward 2", duration_sec=duration, surge=1600),
        MoveRC(name="Turn 2", duration_sec=duration, yaw=1600),
    ]


def body_allaxes(duration):
    pulses = [("surge", 1600), ("sway", 1600), ("heave", 1600),
              ("yaw", 1600), ("pitch", 1550), ("roll", 1550)]
    return [MoveRC(name=f"Pulse {axis}", duration_sec=duration, **{axis: pwm})
            for axis, pwm in pulses]


def body_approach(timeout_sec, label):
    gate = ApproachGate(name=f"Approach '{label}'")
    setattr(gate, "target_label", label)
    return [Timeout(name="Approach (hard cap)", child=gate, duration=timeout_sec)]


def _apply_p_only(depth_ctrl):
    """
    Mutate the depth PID on THIS DepthControl's AxisController so the first
    in-water read is clean: keep kp, zero ki and kd, clear any history.
    Only touches this instance; pid.py is untouched.
    """
    try:
        pid = depth_ctrl.axis_controller.pids["depth"]
        pid.ki = 0.0
        pid.kd = 0.0
        pid.reset()
    except Exception:
        pass


def body_depth(target_depth, timeout_sec, pid_mode):
    depth_ctrl = AdjustDepth(name=f"Hold depth {target_depth}m", target_depth=target_depth)
    if pid_mode == "p_only":
        _apply_p_only(depth_ctrl)
    # Timeout so the mission always ends even if the deadband is never hit
    # (e.g. sensor offset): after timeout the child is preempted (terminate
    # sends STOP), and the sequence proceeds to disarm.
    return [Timeout(name="Depth (hard cap)", child=depth_ctrl, duration=timeout_sec)]


def build_body(p):
    m = p["mission"]
    single = {"surge", "yaw", "heave", "sway", "pitch", "roll"}
    if m in single:
        return body_single(m, p["pwm"], p["duration"])
    if m == "wiggle":
        return body_wiggle(p["duration"])
    if m == "square":
        return body_square(p["duration"])
    if m == "allaxes":
        return body_allaxes(p["duration"])
    if m == "approach":
        return body_approach(p["approach_timeout"], p["approach_label"])
    if m == "depth":
        return body_depth(p["target_depth"], p["depth_timeout"], p["depth_pid_mode"])
    return [MoveRC(name=f"UNKNOWN mission '{m}' (no-op)", duration_sec=p["duration"])]


def create_mission_tree(p):
    """
    [Set ALT_HOLD] -> [Arm] -> <body...> -> [Disarm]
    memory=True: each child runs once, in order.
    Each body child is wrapped in FailureIsSuccess so a FAILURE still lets the
    sequence reach the trailing disarm (never leave the vehicle armed).
    """
    root = Sequence(name=f"BenchTest [{p['mission']}]", memory=True)
    root.add_child(SetFlightMode(name="Set ALT_HOLD", mode="ALT_HOLD"))
    root.add_child(ArmRobot(name="Arm Thrusters", arm=True))

    for child in build_body(p):
        guarded = FailureIsSuccess(name=f"guard::{child.name}", child=child)
        root.add_child(guarded)

    root.add_child(ArmRobot(name="Disarm and STOP", arm=False))
    return root


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("bench_test_node")

    node.declare_parameter("mission", "wiggle")
    node.declare_parameter("pwm", 1600)
    node.declare_parameter("duration", 2.0)
    node.declare_parameter("approach_timeout", 20.0)
    node.declare_parameter("approach_label", "person")
    node.declare_parameter("target_depth", -0.3)
    node.declare_parameter("depth_timeout", 60.0)
    node.declare_parameter("depth_pid_mode", "full")

    g = node.get_parameter
    p = {
        "mission": g("mission").value,
        "pwm": max(1100, min(int(g("pwm").value), 1900)),
        "duration": float(g("duration").value),
        "approach_timeout": float(g("approach_timeout").value),
        "approach_label": g("approach_label").value,
        "target_depth": float(g("target_depth").value),
        "depth_timeout": float(g("depth_timeout").value),
        "depth_pid_mode": g("depth_pid_mode").value,
    }

    # Required so DepthControl / ApproachGate get sensor data on the blackboard.
    bb_manager = BlackboardManager(node=node)  # noqa: F841 (kept alive by ref)

    node.get_logger().info(f"[benchTest] params: {p}")
    node.get_logger().warn(
        "[benchTest] This mission ARMS the vehicle — props will spin. "
        "Confirm props are physically safe.")
    if p["mission"] == "depth":
        node.get_logger().info(
            f"[benchTest] depth: target={p['target_depth']}m "
            f"pid_mode={p['depth_pid_mode']} timeout={p['depth_timeout']}s. "
            "Judge the behaviour; absolute depth may be offset by hardcoded p_atm.")

    root = create_mission_tree(p)
    tree = py_trees_ros.trees.BehaviourTree(root=root)

    try:
        tree.setup(node=node, timeout=15.0)
    except Exception as e:
        node.get_logger().error(f"[benchTest] Critical setup error: {e}")
        rclpy.shutdown()
        return

    print("\n" + "=" * 46)
    print(f"BLUE SPARK benchTest  —  mission: {p['mission']}")
    print(py_trees.display.unicode_tree(root=root))
    print("=" * 46 + "\n")

    node.get_logger().info("[benchTest] Ready — running mission ONCE, then disarm+exit.")

    # Turn SIGTERM into the same KeyboardInterrupt path as Ctrl+C, so `docker
    # stop`, `kill`, etc. also unwind cleanly through finally (-> STOP/disarm).
    def _sigterm(_signum, _frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, _sigterm)

    # Tick the tree OURSELVES instead of tick_tock(). tick_tock loops forever and
    # would re-run the whole mission (arm/move/disarm) again and again. Here we
    # stop as soon as the root reaches SUCCESS or FAILURE — i.e. after ONE run.
    period_s = 0.1
    try:
        while rclpy.ok():
            tree.tick()
            # spin briefly so the node's subscriptions/service responses (arming,
            # blackboard, RC replies) are actually processed between ticks.
            rclpy.spin_once(node, timeout_sec=period_s)

            status = tree.root.status
            if status in (py_trees.common.Status.SUCCESS,
                          py_trees.common.Status.FAILURE):
                node.get_logger().info(
                    f"[benchTest] Mission finished with {status.name}. Exiting.")
                break
    except KeyboardInterrupt:
        node.get_logger().info("[benchTest] Interrupt — unwinding; STOP sent on exit.")
    finally:
        # tree.shutdown() calls terminate() on every behaviour: MoveRC sends STOP
        # (1500) on the axes it touched, ArmRobot(disarm) disarms. This runs on
        # BOTH the normal finish and Ctrl+C/SIGTERM paths.
        try:
            tree.shutdown()
        except Exception as e:
            node.get_logger().error(f"[benchTest] shutdown error: {e}")
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()