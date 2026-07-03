import time


class PID:
    def __init__(
        self,
        kp,
        ki=0.0,
        kd=0.0,
        *,
        name="",
        output_limits=(-400.0, 400.0),
        integral_limit=None,
        derivative_tau=0.0,
        output_sign=1.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.name = name

        self.out_min, self.out_max = output_limits
        if integral_limit is None:
            self.int_min, self.int_max = self.out_min, self.out_max
        else:
            self.int_min, self.int_max = -abs(integral_limit), abs(integral_limit)

        self.derivative_tau = derivative_tau
        self.output_sign = output_sign

        self._integral = 0.0
        self._prev_error = None
        self._prev_deriv = 0.0
        self._prev_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = None
        self._prev_deriv = 0.0
        self._prev_time = None

    def _clamp(self, value, lo, hi):
        return max(lo, min(hi, value))

    def update(self, error, dt=None):
        now = time.monotonic()

        if dt is None:
            dt = 0.0 if self._prev_time is None else now - self._prev_time
        self._prev_time = now

        if dt <= 0.0:
            self._prev_error = error
            return self._finish(self.kp * error)

        p = self.kp * error

        self._integral += self.ki * error * dt
        self._integral = self._clamp(self._integral, self.int_min, self.int_max)
        i = self._integral

        d = 0.0
        if self.kd != 0.0 and self._prev_error is not None:
            raw_deriv = (error - self._prev_error) / dt
            if self.derivative_tau > 0.0:
                alpha = dt / (self.derivative_tau + dt)
                raw_deriv = self._prev_deriv + alpha * (raw_deriv - self._prev_deriv)

            self._prev_deriv = raw_deriv
            d = self.kd * raw_deriv

        self._prev_error = error
        return self._finish(p + i + d)

    def _finish(self, output):
        output *= self.output_sign
        return self._clamp(output, self.out_min, self.out_max)


def make_yaw_pid():
    return PID(
        kp=4.0,
        ki=0.2,
        kd=0.0,
        name="yaw",
        output_limits=(-150.0, 150.0),
        derivative_tau=0.1,
    )


def make_heave_pid():
    return PID(
        kp=4.0,
        ki=0.5,
        kd=0.0,
        name="heave",
        output_limits=(-100.0, 100.0),
        derivative_tau=0.1,
    )


def make_surge_pid():
    return PID(
        kp=100.0,
        ki=5.0,
        kd=0.0,
        name="surge",
        output_limits=(-200.0, 200.0),
        derivative_tau=0.1,
    )


def make_depth_pid():
    return PID(
        kp=200.0,
        ki=10.0,
        kd=20.0,
        name="depth",
        output_limits=(-100.0, 100.0),
        derivative_tau=0.15,
    )


def make_pitch_pid():
    return PID(
        kp=4.0,
        ki=0.2,
        kd=0.0,
        name="pitch",
        output_limits=(-100.0, 100.0),
        derivative_tau=0.1,
    )


def make_roll_pid():
    return PID(
        kp=4.0,
        ki=0.2,
        kd=0.0,
        name="roll",
        output_limits=(-100.0, 100.0),
        derivative_tau=0.1,
    )
