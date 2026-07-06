import py_trees
import time
from std_msgs.msg import Int32
from bluespark_peripherals.utils.types import PayloadAction


class TriggerPayload(py_trees.behaviour.Behaviour):
    """
    Purpose:
        Publishes a single PayloadAction command to the servo node on
        "/payload/action" to fire mission payload mechanisms (dropping
        balls, firing the torpedo) or to reset the servos. After publishing,
        holds the tree in RUNNING for `wait_sec` seconds so the mechanism
        physically completes before the mission continues. This prevents the
        robot from moving away before the payload has been released.

    Usage:
        Add to a Sequence node at the point in the mission where the
        payload should be released.
        Example:
            drop = TriggerPayload(name="Drop Ball 1", action=PayloadAction.DROP_BALL_1)
    """

    def __init__(self, name, action: PayloadAction, wait_sec: float = 5.0):
        super().__init__(name)
        self.action = action
        self.wait_sec = wait_sec
        self.publisher = None
        self.start_time = None
        self.sent = False

    def setup(self, **kwargs):
        try:
            node = kwargs['node']
            self.publisher = node.create_publisher(Int32, '/payload/action', 10)
        except KeyError:
            self.logger.error("ROS 2 node reference missing in setup()!")

    def initialise(self):
        self.logger.info(f"[{self.name}] Requesting payload action: {self.action.name}")
        self.sent = False
        self.start_time = None

        if self.publisher is None:
            self.logger.error(f"[{self.name}] Publisher for /payload/action is unavailable!")
            return

        msg = Int32()
        msg.data = int(self.action)
        self.publisher.publish(msg)
        self.sent = True
        self.start_time = time.time()

    def update(self):
        if not self.sent:
            return py_trees.common.Status.FAILURE

        # Hold the tree while the servo mechanism physically completes,
        # so the robot doesn't move away before the payload is released.
        if time.time() - self.start_time < self.wait_sec:
            return py_trees.common.Status.RUNNING

        self.logger.info(f"[{self.name}] Action '{self.action.name}' completed.")
        return py_trees.common.Status.SUCCESS