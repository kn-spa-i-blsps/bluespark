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

# Import our hardcoded actions (ensure the file is in the same directory)
from behaviours.hardcoded_actions import MoveRC
from behaviours.control import SetFlightMode, ArmRobot
# TODO Move this mission to mission folder, and only run it here
def create_mission_tree():
    """
    Builds the main behavior tree consisting of an action sequence.
    All nodes invoke ROS 2 services asynchronously.
    """
    # The main mission root.
    # memory=True ensures that when a node returns SUCCESS,
    # the tree remembers it and directly ticks the next child.
    mission = Sequence(name="Blue Spark Orbital Mission", memory=True)

    # 1. Pre-flight Setup
    mission.add_child(SetFlightMode(name="Set MANUAL Mode", mode="ALT_HOLD"))
    mission.add_child(ArmRobot(name="Arm Thrusters", arm=True))

    # 2. Dive (Heave < 1500 is downward movement in ArduSub)
    mission.add_child(MoveRC(name="Dive", duration_sec=3.0, heave=1400))

    # 3. Move forward through the gate (Surge > 1500 is forward)
    mission.add_child(MoveRC(name="Forward Through Gate", duration_sec=5.0, surge=1650))

    # 4. Orbit the pole
    # Lateral sway (1600) combined with inward yaw rotation (1450)
    mission.add_child(MoveRC(name="Orbit Pole", duration_sec=7.0, sway=1600, yaw=1450))

    # 5. Turn 180 degrees (Yaw rotation in place)
    mission.add_child(MoveRC(name="Turn 180 Degrees", duration_sec=3.0, yaw=1650))

    # 6. Return straight through the gate
    mission.add_child(MoveRC(name="Return Through Gate", duration_sec=5.0, surge=1650))

    # 7. Surface safely before shutting down the thrusters
    mission.add_child(MoveRC(name="Surface", duration_sec=3.0, heave=1600))

    # 8. Safing / Disarm
    mission.add_child(ArmRobot(name="Disarm and STOP", arm=False))

    return mission


def main(args=None):
    rclpy.init(args=args)

    # 1. Initialize the logic structure
    root_behavior = create_mission_tree()

    # 2. Create the ROS 2 behavior tree manager
    # This automatically instantiates a ROS 2 node in the background
    tree_manager = py_trees_ros.trees.BehaviourTree(root=root_behavior)

    # 3. SETUP Phase
    # We pass the ROS 2 node reference under the 'node' key.
    # This allows the setup() methods in our custom actions to create Service Clients.
    try:
        tree_manager.setup(node=tree_manager.node, timeout=15.0)
    except Exception as e:
        tree_manager.node.get_logger().error(f"Critical setup error: {e}")
        rclpy.shutdown()
        return

    # Display the mission structure in the terminal
    print("\n" + "=" * 40)
    print("BLUE SPARK MISSION STRUCTURE:")
    print(py_trees.display.unicode_tree(root=root_behavior))
    print("=" * 40 + "\n")

    tree_manager.node.get_logger().info('Autonomy node ready! Starting mission execution...')

    try:
        # 4. Execution Loop (Tick-Tock)
        # The tree ticks every 100 ms (10 Hz)
        tree_manager.tick_tock(period_ms=100)
        rclpy.spin(tree_manager.node)
    except KeyboardInterrupt:
        tree_manager.node.get_logger().info("Interrupt signal received (Ctrl+C). Shutting down...")
    finally:
        # 5. Safe shutdown
        # Triggers the terminate() function on the active node (forces PWM to 1500)
        tree_manager.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()