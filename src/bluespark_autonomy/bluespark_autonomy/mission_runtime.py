"""
mission_runtime.py — shared main() for every mission: ONE run, then exit.

A mission file builds its tree and calls run_mission():

    def create_tree(node):          # node: to read parameters (--ros-args -p ...)
        root = Sequence(name="My Mission", memory=True)
        ...
        return root

    def main(args=None):
        run_mission("my_mission", create_tree, args=args)

run_mission():
  - creates the node and the BlackboardManager,
  - sets the tree up and waits until every service the behaviours use exists;
    missing after setup_timeout -> the mission FAILS before doing anything,
  - logs every status change of every behaviour (RUNNING / SUCCESS / FAILURE),
    so the ./bs session timeline shows what the tree did and when,
  - ticks the tree until the ROOT finishes, ONCE, then exits:
        root SUCCESS -> exit code 0
        setup failed -> exit code 2   (services missing)
        root FAILURE -> exit code 3   (a behaviour failed: the mission stops there)
        Ctrl+C       -> exit code 130
    Anything else (e.g. 1 = uncaught Python exception) = the mission crashed.

The mission does not disarm on failure by itself: ./bs mission (mission_runner)
disarms and sets all thrusters to 1500 after every run, whatever the result.
"""

import sys
import time

import py_trees
import py_trees_ros.trees
import rclpy
from rclpy.executors import ExternalShutdownException

from bluespark_autonomy.blackboard_manager import BlackboardManager

EXIT_SUCCESS = 0
EXIT_SETUP_FAILED = 2
EXIT_FAILURE = 3  # not 1: Python exits with 1 on an uncaught exception (crash)
EXIT_INTERRUPTED = 130

DONE = (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE)


class TransitionLogger:
    """Post-tick handler: logs each behaviour whose status changed in this tick.

        [tree] Swim Forward: RUNNING -> SUCCESS
        [tree] Wait for ALT_HOLD: RUNNING -> FAILURE (Pixhawk still in MANUAL after 5 s)
    """

    def __init__(self, logger):
        self.logger = logger
        self.last = {}

    def __call__(self, tree):
        for behaviour in tree.root.iterate():
            old = self.last.get(behaviour.id, py_trees.common.Status.INVALID)
            new = behaviour.status
            if new == old:
                continue
            self.last[behaviour.id] = new
            text = f"[tree] {behaviour.name}: {old.name} -> {new.name}"
            if behaviour.feedback_message:
                text += f" ({behaviour.feedback_message})"
            if new == py_trees.common.Status.FAILURE:
                self.logger.error(text)
            else:
                self.logger.info(text)


def wait_for_services(node, timeout):
    """Wait until every service client of the node sees its server.
    Returns the names of the services still missing after timeout."""
    end = time.monotonic() + timeout
    for client in list(node.clients):
        client.wait_for_service(timeout_sec=max(0.0, end - time.monotonic()))
    return sorted({c.srv_name for c in node.clients if not c.service_is_ready()})


def run_mission(name, create_tree, period_ms=100, setup_timeout=15.0, args=None):
    """Build, set up and tick the mission tree once until the root finishes; exit."""
    rclpy.init(args=args)
    node = rclpy.create_node(f"{name}_node")
    logger = node.get_logger()
    exit_code = EXIT_INTERRUPTED
    tree = None

    try:
        BlackboardManager(node=node)
        root = create_tree(node)
        tree = py_trees_ros.trees.BehaviourTree(root=root)

        try:
            tree.setup(node=node, timeout=setup_timeout)
        except Exception as exc:  # noqa: BLE001 - any setup problem ends the mission
            logger.error(f"Mission setup failed: {exc}")
            exit_code = EXIT_SETUP_FAILED
            return
        missing = wait_for_services(node, setup_timeout)
        if missing:
            logger.error(f"Mission setup failed, services not available: {', '.join(missing)}")
            exit_code = EXIT_SETUP_FAILED
            return

        print("\n" + py_trees.display.unicode_tree(root=root) + "\n", flush=True)
        logger.info(f"Mission '{name}' started")
        tree.add_post_tick_handler(TransitionLogger(logger))
        tree.tick_tock(period_ms=period_ms)

        while rclpy.ok() and root.status not in DONE:
            rclpy.spin_once(node, timeout_sec=0.1)

        if root.status == py_trees.common.Status.SUCCESS:
            logger.info(f"Mission '{name}' finished: SUCCESS")
            exit_code = EXIT_SUCCESS
        elif root.status == py_trees.common.Status.FAILURE:
            logger.error(f"Mission '{name}' finished: FAILURE")
            exit_code = EXIT_FAILURE
    except (KeyboardInterrupt, ExternalShutdownException):
        exit_code = EXIT_INTERRUPTED
    finally:
        if tree is not None:
            try:
                tree.shutdown(destroy_node=False)  # stops ticking
            except Exception:  # noqa: BLE001 - shutting down anyway
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(exit_code)
