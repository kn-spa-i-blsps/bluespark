"""
File: autonomy_node.py
Purpose: The main Executive Node for the autonomy layer.
         Assembles the mission architecture using a Behavior Tree
         and runs the decision loop at 10 Hz.
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

    root_behavior = create_mission_tree()

    tree_manager = py_trees_ros.trees.BehaviourTree(root=root_behavior)

    bb_manager = BlackboardManager(node=tree_manager.node)

    try:
        tree_manager.setup(node=tree_manager.node, timeout=15.0)
    except Exception as e:
        tree_manager.node.get_logger().error(f"Critical setup error: {e}")
        rclpy.shutdown()
        return

    # Wyświetlenie struktury misji w terminalu
    print("\n" + "=" * 40)
    print("BLUE SPARK MISSION STRUCTURE:")
    print(py_trees.display.unicode_tree(root=root_behavior))
    print("=" * 40 + "\n")

    tree_manager.node.get_logger().info('Autonomy node ready! Starting mission execution...')

    try:
        tree_manager.tick_tock(period_ms=100)
        rclpy.spin(tree_manager.node)
    except KeyboardInterrupt:
        tree_manager.node.get_logger().info("Interrupt signal received (Ctrl+C). Shutting down...")
    finally:
        tree_manager.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()