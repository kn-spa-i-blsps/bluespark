"""
File: autonomy_node.py
Purpose: Simple testing a Approaching gate with minimal beahiour tree for it to work
Note: Mostly for simple testing purposes, it can be transformed to a wraper for testing 
    movement actions
"""

import rclpy
import py_trees
import py_trees_ros.trees
from py_trees.composites import Sequence

# Importy zachowań
from bluespark_autonomy.behaviours.control import SetFlightMode, ArmRobot
from bluespark_autonomy.behaviours.actions import ApproachGate

# Import Blackboard Managera
from bluespark_autonomy.blackboard_manager import BlackboardManager

def create_mission_tree():
    """
    Builds the main behavior tree consisting of an action sequence.
    All nodes invoke ROS 2 services asynchronously.
    """
    mission = Sequence(name="Blue Spark Orbital Mission", memory=True)

    # Pre-flight Setup
    mission.add_child(SetFlightMode(name="Set MANUAL Mode", mode="ALT_HOLD"))
    mission.add_child(ArmRobot(name="Arm Thrusters", arm=True))

    #Main mission
    mission.add_child(ApproachGate(name="Approach Gate"))

    # Disarm
    mission.add_child(ArmRobot(name="Disarm and STOP", arm=False))

    return mission


def main(args=None):
    rclpy.init(args=args)

    # Preparing for building and executing a pytree
    node = rclpy.create_node('approacher_test_node') 
    bb_manager = BlackboardManager(node=node)
    # Building behaviour tree
    root_behavior = create_mission_tree()
    tree_manager = py_trees_ros.trees.BehaviourTree(root=root_behavior)

    try:
        tree_manager.setup(node=node, timeout=15.0)
    except Exception as e:
        node.get_logger().error(f"Critical setup error: {e}")
        rclpy.shutdown()
        return

    print("\n" + "=" * 40)
    print("BLUE SPARK APPROACH GATE TEST:")
    print(py_trees.display.unicode_tree(root=root_behavior))
    print("=" * 40 + "\n")

    node.get_logger().info('Autonomy test ready! Starting execution...')

    try:
        tree_manager.tick_tock(period_ms=100)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt signal received. Shutting down...")
    finally:
        tree_manager.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()