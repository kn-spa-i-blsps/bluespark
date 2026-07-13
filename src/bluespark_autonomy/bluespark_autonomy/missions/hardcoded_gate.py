"""
File: hardcodedGate.py
Purpose:
    W pelni hardcodowana misja "przejscie przez bramke" — bez wizji, dead
    reckoning. Delikatnie: krotkie zanurzenie (heave), potem powolny surge do
    przodu przez bramke, potem stop i disarm.

    Kolejnosc (Sequence, memory=True):
        1. SetFlightMode ALT_HOLD
        2. Arm
        3. Dive        — heave 1450 przez 1.0 s   (delikatne zejscie w dol)
        4. Forward     — surge 1560 przez 10.0 s   (powolny przelot przez bramke)
        5. Disarm + STOP

    Konwencja ArduSub (jak w reszcie repo):
        1500 = stop na kazdej osi.
        heave < 1500  -> w dol (nurkowanie).   heave > 1500 -> w gore.
        surge > 1500  -> do przodu.
    MoveRC.terminate() sam wysyla STOP na uzywane osie po kazdym kroku, a
    koncowy ArmRobot(arm=False) rozbraja. Ctrl+C -> tree.shutdown() -> STOP.

    SAFETY: to UZBRAJA pojazd, srigi realnie sie zakrec. Odpalaj z zabezpieczonymi
    srubami. Wartosci sa celowo lagodne (heave 1450, surge 1560), zmien ponizej
    w create_mission_tree, jesli chcesz mocniej/dluzej.

RUN (w kontenerze prod na governerze):
    ros2 run bluespark_autonomy hardcodedGate
    # albo przez bringup:
    ros2 launch bluespark_bringup governer.launch.py autonomy_executable:=hardcodedGate
"""

import rclpy
import py_trees
import py_trees_ros.trees
from py_trees.composites import Sequence

from bluespark_autonomy.behaviours.hardcoded_actions import MoveRC
from bluespark_autonomy.behaviours.control import SetFlightMode, ArmRobot
from bluespark_autonomy.blackboard_manager import BlackboardManager


# ---- Parametry misji (delikatne wartosci; zmien tutaj) -----------------------
DIVE_PWM = 1750        # heave < 1500 = w dol; 1450 = lagodne nurkowanie
DIVE_DURATION = 1.0    # sekundy
DIVE_CONST = 1710
SURGE_PWM = 1600       # surge > 1500 = do przodu; 1560 = powolny przelot
SURGE_DURATION = 10.0  # sekundy


def create_mission_tree():
    """Buduje drzewo: setup -> arm -> dive -> surge -> disarm."""
    mission = Sequence(name="BlueSpark Hardcoded Gate", memory=True)

    # Pre-flight
    mission.add_child(SetFlightMode(name="Set MANUAL Mode", mode="MANUAL"))
    mission.add_child(ArmRobot(name="Arm Thrusters", arm=True))

    # Delikatne zanurzenie
    mission.add_child(MoveRC(
        name="Dive (gentle)", duration_sec=DIVE_DURATION, heave=DIVE_PWM))

    # Powolny przelot przez bramke
    mission.add_child(MoveRC(
        name="Forward Through Gate", duration_sec=SURGE_DURATION, surge=SURGE_PWM, heave=DIVE_CONST))

    # Safing
    mission.add_child(ArmRobot(name="Disarm and STOP", arm=False))

    return mission


def main(args=None):
    rclpy.init(args=args)

    # Recznie tworzymy wezel (jak w approacherTest) — unika problemu z NoneType
    # i daje BlackboardManagerowi gotowy node.
    node = rclpy.create_node('hardcoded_gate_node')

    # BlackboardManager karmi blackboard danymi z sensorow; trzymamy referencje
    # zeby nie zostal zebrany przez GC.
    bb_manager = BlackboardManager(node=node)  # noqa: F841

    root_behavior = create_mission_tree()
    tree_manager = py_trees_ros.trees.BehaviourTree(root=root_behavior)

    try:
        tree_manager.setup(node=node, timeout=15.0)
    except Exception as e:
        node.get_logger().error(f"Critical setup error: {e}")
        rclpy.shutdown()
        return

    print("\n" + "=" * 40)
    print("BLUE SPARK HARDCODED GATE:")
    print(py_trees.display.unicode_tree(root=root_behavior))
    print("=" * 40 + "\n")

    node.get_logger().warn(
        "[hardcodedGate] Ta misja UZBRAJA pojazd — sruby sie zakreca. "
        "Upewnij sie, ze sa fizycznie zabezpieczone.")
    node.get_logger().info('[hardcodedGate] Ready — starting execution...')

    try:
        # Drzewo tyka co 100 ms (10 Hz), spin karmi callbacki.
        tree_manager.tick_tock(period_ms=100)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[hardcodedGate] Interrupt — shutting down; STOP sent on exit.")
    finally:
        tree_manager.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()