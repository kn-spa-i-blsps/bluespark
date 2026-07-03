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

    # 1. Ręczne utworzenie węzła wykonawczego - od razu rozwiązuje problem z NoneType
    node = rclpy.create_node('approacher_test_node')

    # 2. Przekazujemy w 100% gotowy węzeł do Blackboard Managera
    bb_manager = BlackboardManager(node=node)

    # 3. Budujemy logikę drzewa
    root_behavior = create_mission_tree()
    tree_manager = py_trees_ros.trees.BehaviourTree(root=root_behavior)

    # 4. Inicjalizacja drzewa (przekazujemy nasz stworzony węzeł)
    try:
        # Dzięki temu setup() nie musi tworzyć węzła pod maską, tylko używa naszego
        tree_manager.setup(node=node, timeout=15.0)
    except Exception as e:
        node.get_logger().error(f"Critical setup error: {e}")
        rclpy.shutdown()
        return

    # Opcjonalne: Wyświetlenie drzewa w konsoli dla pewności
    print("\n" + "=" * 40)
    print("BLUE SPARK APPROACH GATE TEST:")
    print(py_trees.display.unicode_tree(root=root_behavior))
    print("=" * 40 + "\n")

    node.get_logger().info('Autonomy test ready! Starting execution...')

    try:
        # Odpalamy timer dla drzewa (10 Hz) i kręcimy węzłem, żeby odbierał wiadomości z wizji
        tree_manager.tick_tock(period_ms=100)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupt signal received. Shutting down...")
    finally:
        tree_manager.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()