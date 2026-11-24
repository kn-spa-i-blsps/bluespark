"""
vehicle.py — small synchronous helpers to talk to the running governer stack.

Used by the `vehicle` command (./bs arm / disarm / mode / pwm / neutral / status)
and by mission_runner (pre-flight check + safety stop after every mission).

Arming and modes go straight to mavros (/mavros/cmd/arming, /mavros/set_mode),
so the answer comes from the Pixhawk, and the result is confirmed on
/mavros/state. (/manager/set_arming always answers "success", see TODO.md.)
Thrusters go through rc_override_node (/control/set_<axis>).
"""

import time

import rclpy
from bluespark_interfaces.srv import SetRCOverride
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState

# Index in this tuple = RC channel index in /mavros/rc/override (rc_override_node).
AXES = ("pitch", "roll", "heave", "yaw", "surge", "sway")
NEUTRAL = 1500
PWM_MIN, PWM_MAX = 1100, 1900

# Nodes of the governer stack (bluespark_control + mavros). governer.launch.py
# names all three nodes of vehicle_manager_node "vehicle_manager_node".
STACK_NODES = ("mavros", "rc_override_node", "vehicle_manager_node")

SENSOR_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


class Vehicle:
    def __init__(self, node):
        self.node = node
        self.state = None
        self.rc = None
        self.battery = None
        node.create_subscription(State, "/mavros/state", self._on_state, SENSOR_QOS)
        node.create_subscription(OverrideRCIn, "/mavros/rc/override", self._on_rc, SENSOR_QOS)
        node.create_subscription(BatteryState, "/mavros/battery", self._on_battery, SENSOR_QOS)
        self.arming = node.create_client(CommandBool, "/mavros/cmd/arming")
        self.mode = node.create_client(SetMode, "/mavros/set_mode")
        self.axes = {a: node.create_client(SetRCOverride, f"/control/set_{a}") for a in AXES}

    # --- topics ---
    def _on_state(self, msg):
        self.state = msg

    def _on_rc(self, msg):
        self.rc = msg

    def _on_battery(self, msg):
        self.battery = msg

    def spin_until(self, condition, timeout):
        """Spin the node until condition() is true or timeout (s). Returns condition()."""
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def wait_state(self, timeout=3.0):
        self.spin_until(lambda: self.state is not None, timeout)
        return self.state

    # --- services ---
    def _call_all(self, calls, timeout):
        """calls: {key: (client, request)}. Sends all at once, waits for all.
        Returns {key: response or error string}."""
        results = {}
        futures = {}
        end = time.monotonic() + timeout
        for key, (client, request) in calls.items():
            if not client.wait_for_service(timeout_sec=max(0.0, end - time.monotonic())):
                results[key] = f"service {client.srv_name} not available"
                continue
            futures[key] = client.call_async(request)
        self.spin_until(lambda: all(f.done() for f in futures.values()), max(0.0, end - time.monotonic()))
        for key, future in futures.items():
            if not future.done():
                results[key] = "no answer (timeout)"
            elif future.exception() is not None:
                results[key] = f"error: {future.exception()}"
            else:
                results[key] = future.result()
        return results

    def set_armed(self, armed, timeout=5.0):
        """Returns (ok, text). ok = Pixhawk accepted AND /mavros/state confirms."""
        word = "arm" if armed else "disarm"
        res = self._call_all({"x": (self.arming, CommandBool.Request(value=armed))}, timeout)["x"]
        if isinstance(res, str):
            return False, f"{word}: {res}"
        if not res.success:
            return False, f"{word}: rejected by the Pixhawk (MAV_RESULT {res.result})"
        if not self.spin_until(lambda: self.state is not None and self.state.armed == armed, 3.0):
            return False, f"{word}: accepted, but /mavros/state does not confirm it"
        return True, "armed" if armed else "disarmed"

    def set_mode(self, mode, timeout=5.0):
        mode = mode.upper()
        res = self._call_all({"x": (self.mode, SetMode.Request(custom_mode=mode))}, timeout)["x"]
        if isinstance(res, str):
            return False, f"mode {mode}: {res}"
        if not res.mode_sent:
            return False, f"mode {mode}: not sent (unknown mode name?)"
        if not self.spin_until(lambda: self.state is not None and self.state.mode == mode, 3.0):
            current = self.state.mode if self.state else "?"
            return False, f"mode {mode}: sent, but the Pixhawk stays in {current}"
        return True, f"mode {mode}"

    def set_pwm(self, values, timeout=3.0):
        """values: {axis: pwm}. Returns (ok, text)."""
        calls = {
            axis: (self.axes[axis], SetRCOverride.Request(pwm_value=int(pwm)))
            for axis, pwm in values.items()
        }
        results = self._call_all(calls, timeout)
        failed = {
            axis: (r if isinstance(r, str) else "rejected")
            for axis, r in results.items()
            if isinstance(r, str) or not r.success
        }
        if failed:
            return False, "pwm: " + ", ".join(f"{a} {why}" for a, why in failed.items())
        return True, "pwm " + " ".join(f"{a}={values[a]}" for a in values)

    def neutral(self):
        ok, _ = self.set_pwm({axis: NEUTRAL for axis in AXES})
        return ok, ("thrusters neutral (1500)" if ok else "thrusters NOT set to neutral")

    def safe_stop(self):
        """Disarm first (stops the thrusters immediately), then neutral PWM so
        a later arm does not jump. Returns [(ok, text), ...]."""
        return [self.set_armed(False), self.neutral()]

    def stack_nodes(self, wait=4.0):
        """Returns (present, missing) names from STACK_NODES. Waits up to `wait`
        seconds for discovery (a fresh process does not see every node at once)."""
        self.spin_until(lambda: set(STACK_NODES) <= set(self.node.get_node_names()), wait)
        names = set(self.node.get_node_names())
        present = [n for n in STACK_NODES if n in names]
        missing = [n for n in STACK_NODES if n not in names]
        return present, missing


def check_pwm(axis, value):
    """Raises ValueError with a readable message for a bad axis / value."""
    if axis not in AXES:
        raise ValueError(f"unknown axis '{axis}', use one of: {', '.join(AXES)}")
    if not PWM_MIN <= value <= PWM_MAX:
        raise ValueError(f"PWM {value} out of range {PWM_MIN}-{PWM_MAX} (1500 = stop)")
