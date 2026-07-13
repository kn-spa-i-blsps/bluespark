"""
File: benchTest.py
Purpose:
    Bench-test entry point for single open-loop moves, based directly on the
    working hardcoded_mission.py. ONLY MoveRC + arm/disarm — no vision, no depth,
    no BlackboardManager (those pulled in unrelated, half-finished behaviours
    that crash on import/init).

    The ONLY behavioural change vs the working mission: the tick loop stops
    after ONE pass (root SUCCESS/FAILURE) instead of tick_tock() re-running the
    whole mission forever.

    Pick the mission with a ROS param:
        ros2 run bluespark_autonomy benchTest --ros-args -p mission:=wiggle
        ros2 run bluespark_autonomy benchTest --ros-args -p mission:=surge -p pwm:=1600 -p duration:=2.0
        ros2 run bluespark_autonomy benchTest --ros-args -p mission:=square

    Missions:
        wiggle  - yaw left, stop, yaw right (smallest sanity check).
        surge/heave/yaw/sway/pitch/roll - one push on that axis, then stop.
        square  - surge/yaw/surge/yaw (multi-step sequence in order).

    SAFETY: arms the vehicle; props spin. Run with props safe. MoveRC.terminate()
    sends 1500 (STOP) on exit; trailing ArmRobot(disarm) disarms.
"""

import rclpy
import py_trees
import py_trees_ros.trees
from py_trees.composites import Sequence

from bluespark_autonomy.behaviours.hardcoded_actions import MoveRC
from bluespark_autonomy.behaviours.control import SetFlightMode, ArmRobot


def build_body(mission, pwm, duration):
    single = {"surge", "yaw", "heave", "sway", "pitch", "roll"}
    if mission in single:
        return [MoveRC(name=f"{mission} @ {pwm}", duration_sec=duration, **{mission: pwm})]
    if mission == "wiggle":
        return [
            MoveRC(name="Yaw left", duration_sec=duration, yaw=1450),
            MoveRC(name="Center", duration_sec=1.0),
            MoveRC(name="Yaw right", duration_sec=duration, yaw=1550),
        ]
    if mission == "square":
        return [
            MoveRC(name="Forward 1", duration_sec=duration, surge=1600),
            MoveRC(name="Turn 1", duration_sec=duration, yaw=1600),
            MoveRC(name="Forward 2", duration_sec=duration, surge=1600),
            MoveRC(name="Turn 2", duration_sec=duration, yaw=1600),
        ]
    return [MoveRC(name=f"UNKNOWN '{mission}' (no-op)", duration_sec=duration)]


def create_mission_tree(mission, pwm, duration):
    root = Sequence(name=f"BenchTest [{mission}]", memory=True)
    root.add_child(SetFlightMode(name="Set ALT_HOLD", mode="ALT_HOLD"))
    root.add_child(ArmRobot(name="Arm Thrusters", arm=True))
    for child in build_body(mission, pwm, duration):
        root.add_child(child)
    root.add_child(ArmRobot(name="Disarm and STOP", arm=False))
    return root


def main(args=None):
    rclpy.init(args=args)

    # Create our own node FIRST (tree_manager.node is None until setup()).
    node = rclpy.create_node("bench_test_node")

    node.declare_parameter("mission", "wiggle")
    node.declare_parameter("pwm", 1600)
    node.declare_parameter("duration", 2.0)
    mission = node.get_parameter("mission").get_parameter_value().string_value
    pwm = max(1100, min(int(node.get_parameter("pwm").get_parameter_value().integer_value), 1900))
    duration = float(node.get_parameter("duration").get_parameter_value().double_value)

    root_behavior = create_mission_tree(mission, pwm, duration)
    tree_manager = py_trees_ros.trees.BehaviourTree(root=root_behavior)

    try:
        # Pass our node in, exactly like approacherTest.py does.
        tree_manager.setup(node=node, timeout=15.0)
    except Exception as e:
        node.get_logger().error(f"Critical setup error: {e}")
        rclpy.shutdown()
        return

    print("\n" + "=" * 40)
    print(f"BLUE SPARK benchTest — mission: {mission}")
    print(py_trees.display.unicode_tree(root=root_behavior))
    print("=" * 40 + "\n")

    node.get_logger().info(
        f"benchTest ready (mission={mission}, pwm={pwm}, duration={duration}s). "
        "Running ONCE, then disarm+exit.")

    try:
        # ONLY CHANGE vs hardcoded_mission.py: instead of tick_tock() looping
        # forever, tick one iteration at a time and stop once the root finishes.
        while rclpy.ok():
            tree_manager.tick_tock(period_ms=100, number_of_iterations=1)
            status = root_behavior.status
            if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE):
                node.get_logger().info(f"Mission finished ({status.name}). Exiting.")
                break
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt (Ctrl+C). Shutting down...")
    finally:
        tree_manager.shutdown()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()