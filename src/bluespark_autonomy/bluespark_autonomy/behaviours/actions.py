import py_trees
from math import hypot
from bluespark_interfaces.srv import SetRCOverride
from bluespark_autonomy.pid.axis_controller import AxisController


class BaseRCBehaviour(py_trees.behaviour.Behaviour):

    def __init__(self, name: str, active_axes: list):
        super().__init__(name=name)
        self.active_axes = active_axes
        self.clients = {}
        self.axis_controller = AxisController()

    def setup(self, **kwargs):
        try:
            node = kwargs['node']
            for axis in self.active_axes:
                self.clients[axis] = node.create_client(SetRCOverride, f'control/set_{axis}')
        except KeyError:
            self.logger.error(f"[{self.name}] ROS 2 node reference missing in setup()!")

    def _send_single_pwm(self, axis, pwm_value):
        if axis in self.clients and self.clients[axis].service_is_ready():
            req = SetRCOverride.Request(pwm_value=int(pwm_value))
            self.clients[axis].call_async(req)

    def terminate(self, new_status):
        for axis in self.active_axes:
            self._send_single_pwm(axis, self.axis_controller.STOP_PWM)


class ApproachGate(BaseRCBehaviour):
    def __init__(self, name: str):
        # Rejestrujemy tylko te 3 osie, których fizycznie tu używamy
        super().__init__(name=name, active_axes=["yaw", "heave", "surge"])

        self.vision_bb = self.attach_blackboard_client(name=self.name, namespace="vision")
        self.vision_bb.register_key(key="detected_objects", access=py_trees.common.Access.READ)

        self.STATE_SEARCHING = "SEARCHING"
        self.STATE_CENTERING = "CENTERING"
        self.STATE_APPROACHING = "APPROACHING"

        self.current_state = self.STATE_SEARCHING

    def initialise(self):
        pass

    def update(self):
        detected_objects = self.vision_bb.detected_objects if self.vision_bb.exists("detected_objects") else {}

        if not detected_objects:
            self._stop_and_search()
            return py_trees.common.Status.RUNNING

        target = None
        for obj in detected_objects.values():
            if obj.label == "person":
                target = obj
                break

        if target is None:
            self._stop_and_search()
            return py_trees.common.Status.RUNNING

        TARGET_DISTANCE = 1.0
        DEADBAND_ANGLE = 10.0

        error_yaw = target.cam_h_angle_deg
        error_heave = target.cam_v_angle_deg
        error_dist = hypot(target.pos_x, target.pos_y, target.pos_z) - TARGET_DISTANCE

        # TODO: return success if target distance is achieved

        is_centered = (abs(error_yaw) < DEADBAND_ANGLE) and (abs(error_heave) < DEADBAND_ANGLE)

        self.current_state = self.STATE_APPROACHING if is_centered else self.STATE_CENTERING

        if self.current_state == self.STATE_CENTERING:
            self._send_single_pwm("surge", self.axis_controller.STOP_PWM)

            if abs(error_yaw) > DEADBAND_ANGLE:
                self._send_single_pwm("yaw", self.axis_controller.update("yaw", error_yaw))
            else:
                self._send_single_pwm("yaw", self.axis_controller.STOP_PWM)
            if abs(error_heave) > DEADBAND_ANGLE:
                self._send_single_pwm("heave", self.axis_controller.update("heave", error_heave))
            else:
                self._send_single_pwm("heave", self.axis_controller.STOP_PWM)

        elif self.current_state == self.STATE_APPROACHING:
            self._send_single_pwm("yaw", self.axis_controller.STOP_PWM)
            self._send_single_pwm("heave", self.axis_controller.STOP_PWM)
            self._send_single_pwm("surge", self.axis_controller.update("surge", error_dist))

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        super().terminate(new_status)
        self.axis_controller.reset_all()

    def _stop_and_search(self):
        self.current_state = self.STATE_SEARCHING
        self._send_single_pwm("surge", self.axis_controller.STOP_PWM)
        self._send_single_pwm("heave", self.axis_controller.STOP_PWM)
        self._send_single_pwm("yaw", 1550)


class DepthControl(BaseRCBehaviour):
    def __init__(self, name, target_depth):
        super().__init__(name=name, active_axes=["heave"])

        self.depth_bb = self.attach_blackboard_client(name=self.name, namespace="depth")
        self.depth_bb.register_key(key="current_depth", access=py_trees.common.Access.READ)

        self.target_depth = target_depth
        self.DEADBAND_DEPTH = 0.15

    def initialise(self):
        pass

    def update(self):
        if not self.depth_bb.exists("current_depth"):
            return py_trees.common.Status.RUNNING

        depth_error = self.target_depth - self.depth_bb.current_depth

        if abs(depth_error) < self.DEADBAND_DEPTH:
            self._send_single_pwm("heave", self.axis_controller.STOP_PWM)
            return py_trees.common.Status.SUCCESS
        else:
            heave_pwm = self.axis_controller.update("heave", depth_error)
            self._send_single_pwm("heave", heave_pwm)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        super().terminate(new_status)
        self.axis_controller.reset_all()