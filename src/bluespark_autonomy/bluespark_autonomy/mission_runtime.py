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
  - sets the tree up, then waits until the FCU is connected (/mavros/state) and
    every service the behaviours use exists. The launch starts the mission
    together with mavros and the control nodes, so this wait is what orders the
    startup. setup_timeout is ONE budget for all of it (tree setup + FCU +
    services); not ready in time -> the mission FAILS before doing anything.
    It is a ROS parameter (-p setup_timeout:=45.0, declared by py_trees_ros),
    default: the run_mission() argument,
  - logs every status change of every behaviour (RUNNING / SUCCESS / FAILURE),
    so the ./bs session timeline shows what the tree did and when,
  - ticks the tree until the ROOT finishes, ONCE, then exits:
        root SUCCESS -> exit code 0
        setup failed -> exit code 2   (FCU not connected / services missing)
        root FAILURE -> exit code 3   (a behaviour failed: the mission stops there)
        Ctrl+C       -> exit code 130
        crash        -> exit code 1   (uncaught exception, traceback in the log)

The mission does not disarm on failure by itself: ./bs mission (mission_runner)
disarms and sets all thrusters to 1500 after every run, whatever the result.
"""

import sys
import time
import traceback

import py_trees
import py_trees_ros.trees
import rclpy
from mavros_msgs.msg import State
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future

from bluespark_autonomy.blackboard_manager import BlackboardManager

EXIT_SUCCESS = 0
EXIT_CRASHED = 1  # uncaught exception, like plain Python
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


def wait_for_fcu(node, timeout):
    """Spin until /mavros/state says connected. Returns True if it did in time."""
    connected = Future()

    def on_state(msg):
        if msg.connected and not connected.done():
            connected.set_result(True)

    # mavros publishes it latched and only on heartbeats / changes, so subscribe
    # the same way to get the current state at once.
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    sub = node.create_subscription(State, "/mavros/state", on_state, qos)
    try:
        # Returns on connected, timeout or rclpy shutdown (Ctrl+C).
        rclpy.spin_until_future_complete(node, connected, timeout_sec=timeout)
    finally:
        node.destroy_subscription(sub)
    return connected.done()


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
    exit_code = EXIT_CRASHED  # until proven otherwise
    tree = None

    try:
        BlackboardManager(node=node)
        root = create_tree(node)
        tree = py_trees_ros.trees.BehaviourTree(root=root)

        start = time.monotonic()  # setup_timeout covers ALL setup steps from here
        try:
            tree.setup(node=node, timeout=setup_timeout)
        except Exception as exc:  # noqa: BLE001 - any setup problem ends the mission
            logger.error(f"Mission setup failed: {exc}")
            exit_code = EXIT_SETUP_FAILED
            return
        # py_trees_ros declares the setup_timeout parameter in setup() (default:
        # our argument); a value from the launch / command line wins. Read it
        # back so tree setup + FCU + services share that one budget.
        setup_timeout = node.get_parameter("setup_timeout").get_parameter_value().double_value
        end = start + setup_timeout
        logger.info(f"Waiting for the FCU and services (up to {setup_timeout:g} s)...")
        if not wait_for_fcu(node, max(0.0, end - time.monotonic())):
            if not rclpy.ok():
                exit_code = EXIT_INTERRUPTED
                return
            logger.error(
                f"Mission setup failed: FCU not connected after {setup_timeout:g} s (/mavros/state)")
            exit_code = EXIT_SETUP_FAILED
            return
        missing = wait_for_services(node, max(0.0, end - time.monotonic()))
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
        else:  # rclpy shut down (Ctrl+C) between two spins
            exit_code = EXIT_INTERRUPTED
    except (KeyboardInterrupt, ExternalShutdownException):
        exit_code = EXIT_INTERRUPTED
    except Exception:  # noqa: BLE001 - report it, then exit with EXIT_CRASHED
        # Without this the sys.exit() below would swallow the traceback.
        logger.error(f"Mission '{name}' crashed:\n{traceback.format_exc()}")
        exit_code = EXIT_CRASHED
    finally:
        if tree is not None:
            try:
                tree.shutdown(destroy_node=False)  # stops ticking
            except Exception:  # noqa: BLE001 - shutting down anyway
                pass
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(exit_code)
