import py_trees
from math import hypot
from bluespark_interfaces.srv import SetRCOverride
from bluespark_autonomy.control_state import ControlState

class ApproachGate(py_trees.behaviour.Behaviour):
    def __init__(self, name: str):
        super().__init__(name=name)

        self.vision_bb = self.attach_blackboard_client(name=self.name, namespace="vision")
        self.vision_bb.register_key(
            key="detected_objects",
            access=py_trees.common.Access.READ
        )

        self.STATE_SEARCHING = "SEARCHING"
        self.STATE_CENTERING = "CENTERING"
        self.STATE_APPROACHING = "APPROACHING"

        self.current_state = self.STATE_SEARCHING
        self.control_state = ControlState()
        self.clients = {}

    def setup(self, **kwargs):
        try:
            node = kwargs['node']
            self.clients = {
                "pitch": node.create_client(SetRCOverride, 'control/set_pitch'),
                "roll": node.create_client(SetRCOverride, 'control/set_roll'),
                "heave": node.create_client(SetRCOverride, 'control/set_heave'),
                "yaw": node.create_client(SetRCOverride, 'control/set_yaw'),
                "surge": node.create_client(SetRCOverride, 'control/set_surge'),
                "sway": node.create_client(SetRCOverride, 'control/set_sway')
            }
        except KeyError:
            self.logger.error("ROS 2 node reference missing in setup()!")

    def initialise(self):
        pass

    def update(self):
        detected_objects = {}
        if self.vision_bb.exists("detected_objects"):
            detected_objects = self.vision_bb.detected_objects

        if not detected_objects:
            self._stop_and_search()
            return py_trees.common.Status.RUNNING

        target = None
        # TODO: Maybe add something if more than one gate detected
        for obj in detected_objects.values():
            if obj.label == "person": # FIXME: add real gate label
                target = obj
                break

        if target is None:
            self._stop_and_search()
            return py_trees.common.Status.RUNNING

        TARGET_DISTANCE = 1.0
        DEADBAND_ANGLE = 10.0

        Kp_yaw = 4.0
        Kp_heave = 4.0
        Kp_surge = 100.0

        error_yaw = target.cam_h_angle_deg
        error_heave = target.cam_v_angle_deg

        curr_dist = hypot(target.pos_x, target.pos_y, target.pos_z)
        error_dist = curr_dist - TARGET_DISTANCE

        is_centered = ((abs(error_yaw) < DEADBAND_ANGLE)
                       and (abs(error_heave) < DEADBAND_ANGLE))

        if not is_centered:
            self.current_state = self.STATE_CENTERING
        else:
            self.current_state = self.STATE_APPROACHING

        if self.current_state == self.STATE_CENTERING:
            self.control_state.set_pwm("surge", ControlState.STOP_PWM)

            if abs(error_yaw) > DEADBAND_ANGLE:
                yaw_pwm = 1500 + (error_yaw * Kp_yaw)
                self.control_state.set_pwm("yaw", yaw_pwm)
            else:
                self.control_state.set_pwm("yaw", ControlState.STOP_PWM)

            if abs(error_heave) > DEADBAND_ANGLE:
                heave_pwm = 1500 + (error_heave * Kp_heave)
                self.control_state.set_pwm("heave", heave_pwm)
            else:
                self.control_state.set_pwm("heave", ControlState.STOP_PWM)

        elif self.current_state == self.STATE_APPROACHING:
            self.control_state.set_pwm("yaw", ControlState.STOP_PWM)
            self.control_state.set_pwm("heave", ControlState.STOP_PWM)

            surge_pwm = 1500 + (error_dist * Kp_surge)
            self.control_state.set_pwm("surge", surge_pwm)

        self._send_rc_overrides()
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        for axis in self.clients.keys():
            self._send_single_pwm(axis, ControlState.STOP_PWM)

    def _stop_and_search(self):
        self.current_state = self.STATE_SEARCHING

        self.control_state.set_pwm("surge", ControlState.STOP_PWM)
        self.control_state.set_pwm("sway", ControlState.STOP_PWM)
        self.control_state.set_pwm("heave", ControlState.STOP_PWM)
        self.control_state.set_pwm("pitch", ControlState.STOP_PWM)
        self.control_state.set_pwm("roll", ControlState.STOP_PWM)

        self.control_state.set_pwm("yaw", 1550)
        self._send_rc_overrides()

    def _send_rc_overrides(self):
        changes = self.control_state.get()
        for axis, pwm in changes.items():
            self._send_single_pwm(axis, pwm)

    def _send_single_pwm(self, axis, pwm_value):
        if axis in self.clients and self.clients[axis].service_is_ready():
            req = SetRCOverride.Request(pwm_value=int(pwm_value))
            self.clients[axis].call_async(req)