from bluespark_autonomy.pid.pid import (
    make_depth_pid,
    make_heave_pid,
    make_pitch_pid,
    make_roll_pid,
    make_surge_pid,
    make_yaw_pid,
)

STOP_PWM = 1500


class AxisController:
    def __init__(self, control_hz=10.0):
        self.pids = {
            "yaw": make_yaw_pid(),
            "heave": make_heave_pid(),
            "surge": make_surge_pid(),
            "pitch": make_pitch_pid(),
            "roll": make_roll_pid(),
            "depth": make_depth_pid(),
        }
        self.dt = 1.0 / control_hz

    def update(self, axis, error):
        bias = self.pids[axis].update(error, self.dt)
        return self._to_pwm(bias)

    def _to_pwm(self, bias):
        pwm = STOP_PWM + bias
        return int(max(1100, min(1900, pwm)))

    def reset(self, axis):
        self.pids[axis].reset()

    def reset_all(self):
        for pid in self.pids.values():
            pid.reset()
