"""
File: depthHoldTest.py
Purpose: Depth hold test. A SuccessIsRunning decorator keeps DepthControl
    running at the target depth indefinitely - no timeout, the mission ends on
    Ctrl+C only.

    On exit the tree is halted (heave 1500) and, when skip_fcu is false, ArmRobot
    disarms the vehicle so Ctrl+C never leaves it armed.

Note: Depth is negative underwater (see BlackboardManager._depth_callback), so
    target_depth=-1.0 means "one metre below the surface".

Usage:
    ros2 run bluespark_autonomy depthHoldTest
    ros2 run bluespark_autonomy depthHoldTest --ros-args -p target_depth:=-2.0
    ros2 run bluespark_autonomy depthHoldTest --ros-args -p skip_fcu:=true
"""

import signal
import time

import rclpy
import py_trees
import py_trees.decorators
import py_trees_ros.trees
from py_trees.composites import Sequence
from rclpy.signals import SignalHandlerOptions

# Importy zachowań
from bluespark_autonomy.behaviours.control import SetFlightMode, ArmRobot
from bluespark_autonomy.behaviours.actions import DepthControl

# Import Blackboard Managera
from bluespark_autonomy.blackboard_manager import BlackboardManager


def create_mission_tree(target_depth, skip_fcu=False):
    """
    Builds a tree that holds depth until it is halted. SetFlightMode/ArmRobot
    need mavros, so they are only added when skip_fcu is false (bench test
    without a Pixhawk). There is no Disarm child: it would never be reached,
    disarming happens in main() on exit.
    """
    mission = Sequence(name="Blue Spark Depth Hold Test", memory=True)

    if not skip_fcu:
        mission.add_child(SetFlightMode(name="Set ALT_HOLD Mode", mode="ALT_HOLD"))
        mission.add_child(ArmRobot(name="Arm Thrusters", arm=True))

    # SuccessIsRunning maps DepthControl's "reached the deadband" SUCCESS to
    # RUNNING, so py_trees re-initialises it on the next tick and keeps trimming
    # depth forever instead of ending the mission.
    mission.add_child(
        py_trees.decorators.SuccessIsRunning(
            name=f"Hold {target_depth:.1f} m",
            child=DepthControl(name="DepthControl", target_depth=target_depth),
        )
    )

    return mission


def main(args=None):
    # Handle SIGINT/SIGTERM ourselves (SignalHandlerOptions.NO) - rclpy's default
    # handler would shut the context down before we get a chance to disarm.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    stop_requested = {"value": False}

    def handle_signal(signum, frame):
        stop_requested["value"] = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    node = rclpy.create_node('depth_hold_test_node')

    target_depth = node.declare_parameter('target_depth', -1.0).value
    skip_fcu = node.declare_parameter('skip_fcu', False).value

    bb_manager = BlackboardManager(node=node)
    root_behavior = create_mission_tree(target_depth=target_depth, skip_fcu=skip_fcu)
    tree_manager = py_trees_ros.trees.BehaviourTree(root=root_behavior)

    try:
        tree_manager.setup(node=node, timeout=15.0)
    except Exception as e:
        node.get_logger().error(f"Critical setup error: {e}")
        rclpy.try_shutdown()
        return

    print("\n" + "=" * 40)
    print("BLUE SPARK DEPTH HOLD TEST:")
    print(py_trees.display.unicode_tree(root=root_behavior))
    print("=" * 40 + "\n")

    node.get_logger().info(
        f'Depth hold ready! Target depth: {target_depth} m '
        f'(Ctrl+C stops the tree and disarms)'
    )

    try:
        tree_manager.tick_tock(period_ms=100)
        while rclpy.ok() and not stop_requested["value"]:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info("Interrupt received. Halting the tree...")
    finally:
        # destroy_node=False keeps the node alive so ArmRobot can still disarm.
        tree_manager.shutdown(destroy_node=False)  # halts DepthControl -> heave 1500
        if not skip_fcu:
            disarm = ArmRobot(name="Disarm and STOP", arm=False)
            disarm.setup(node=node)
            disarm.initialise()
            deadline = time.monotonic() + 2.0
            while (
                rclpy.ok()
                and disarm.update() == py_trees.common.Status.RUNNING
                and time.monotonic() < deadline
            ):
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
