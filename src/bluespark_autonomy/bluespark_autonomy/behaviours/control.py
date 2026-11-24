import time

import py_trees
from mavros_msgs.srv import CommandBool, SetMode

class ArmRobot(py_trees.behaviour.Behaviour):
    """
    Purpose:
        Asynchronously arms or disarms the robot's thrusters using the
        direct /mavros/cmd/arming service.

    Usage:
        Place after SetFlightMode in the startup sequence, and at the end
        of the mission sequence with arm=False to safely lock the thrusters.
        Example:
            arm_drone = ArmRobot(name="Arm Thrusters", arm=True)
    """

    def __init__(self, name, arm=True):
        super().__init__(name)
        self.arm = arm
        self.state_bb = self.attach_blackboard_client(name=self.name, namespace="state")
        self.state_bb.register_key(key="is_armed", access=py_trees.common.Access.READ)
        self.client = None
        self.future = None

    def setup(self, **kwargs):
        try:
            node = kwargs['node']
            self.client = node.create_client(CommandBool, '/mavros/cmd/arming')
        except KeyError:
            self.logger.error("ROS 2 node reference missing in setup()!")


    def initialise(self):
        action_str = "Arming" if self.arm else "Disarming"
        self.logger.info(f"[{self.name}] Requesting: {action_str} thrusters...")
        self.future = None

        if self.state_bb.exists("is_armed"):
            if self.state_bb.is_armed == self.arm:
                self.logger.info(f"[{self.name}] Thrusters are already {action_str}ed.")
                return

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.error(f"[{self.name}] Service /mavros/cmd/arming is unavailable!")
            return

        req = CommandBool.Request()
        req.value = self.arm
        self.future = self.client.call_async(req)

    def update(self):
        if self.state_bb.exists("is_armed") and self.state_bb.is_armed == self.arm:
            return py_trees.common.Status.SUCCESS

        if self.future is None:
            return py_trees.common.Status.FAILURE

        if self.future.done():
            response = self.future.result()
            if response.success:
                self.logger.info(f"[{self.name}] Action confirmed by the controller.")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"[{self.name}] Mavros command rejected.")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING



class SetFlightMode(py_trees.behaviour.Behaviour):
    """
    Purpose:
        Sends an asynchronous request to mavros on "/mavros/set_mode" to change
        the flight mode. Defaults to MANUAL for RC Override tests.
        Does not block the tree execution while waiting for the service response.

    Usage:
        Add to a Sequence node at the very beginning of the mission tree.
        Example:
            set_mode = SetFlightMode(name="Set MANUAL Mode", mode="MANUAL")
    """
    # TODO ckeck what even manual is (now manual works at krowa's gazebo)

    def __init__(self, name, mode):
        super().__init__(name)
        self.mode = mode
        self.state_bb = self.attach_blackboard_client(name=self.name, namespace="state")
        self.state_bb.register_key(key="flight_mode", access=py_trees.common.Access.READ)
        self.client = None
        self.future = None

    def setup(self, **kwargs):
        try:
            node = kwargs['node']
            self.client = node.create_client(SetMode, '/mavros/set_mode')
        except KeyError:
            self.logger.error("ROS 2 node reference missing in setup()!")

    def initialise(self):
        self.logger.info(f"[{self.name}] Requesting flight mode change to: {self.mode}")
        self.future = None

        if self.state_bb.exists("flight_mode") and self.state_bb.flight_mode == self.mode:
            self.logger.info(f"[{self.name}] Flight mode is already {self.mode}.")
            return

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.logger.error(f"[{self.name}] Service /mavros/set_mode is unavailable!")
            return

        req = SetMode.Request()
        req.custom_mode = self.mode
        self.future = self.client.call_async(req)

    def update(self):

        if self.state_bb.exists("flight_mode") and self.state_bb.flight_mode == self.mode:
            return py_trees.common.Status.SUCCESS

        if self.future is None:
            return py_trees.common.Status.FAILURE


        if self.future.done():
            response = self.future.result()
            if response.mode_sent:
                self.logger.info(f"[{self.name}] Mode '{self.mode}' successfully set.")
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.error(f"[{self.name}] Flight controller rejected the mode change.")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING


class WaitForMode(py_trees.behaviour.Behaviour):
    """
    Purpose:
        Confirms that the Pixhawk is REALLY in the given flight mode (read from
        /mavros/state via the blackboard). SetFlightMode only knows that the
        request was sent; the Pixhawk may still refuse it, e.g. ALT_HOLD without
        a working depth sensor.

    Usage:
        Put right after SetFlightMode. FAILURE after timeout_sec, so the mission
        stops there instead of moving in the wrong mode.
        Example:
            WaitForMode(name="Confirm ALT_HOLD", mode="ALT_HOLD", timeout_sec=5.0)
    """

    def __init__(self, name, mode, timeout_sec=5.0):
        super().__init__(name)
        self.mode = mode
        self.timeout_sec = timeout_sec
        self.state_bb = self.attach_blackboard_client(name=self.name, namespace="state")
        self.state_bb.register_key(key="flight_mode", access=py_trees.common.Access.READ)
        self.start_time = None

    def initialise(self):
        self.start_time = time.monotonic()

    def update(self):
        current = self.state_bb.flight_mode if self.state_bb.exists("flight_mode") else "?"
        if current == self.mode:
            self.feedback_message = f"Pixhawk in {self.mode}"
            return py_trees.common.Status.SUCCESS
        if time.monotonic() - self.start_time >= self.timeout_sec:
            self.feedback_message = (
                f"Pixhawk still in {current} after {self.timeout_sec:g} s, not {self.mode}"
            )
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING
