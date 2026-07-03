# PID control

Two classes: `PID` is the engine - one controller for one axis.
`AxisController` holds one `PID` per axis
(yaw, heave, surge, pitch, roll, depth) and returns
a ready to send PWM value.
The node talks to `AxisController` but neither
class sends anything to a motor.

## P / PI / PID

One class covers all three - disable a term
by zeroing its gain.

| controller | how              |
|------------|------------------|
| P          | ki = 0, kd = 0   |
| PI         | kd = 0           |
| PID        | all three != 0   |

## What it does

The controller turns an error into a command
(a PWM bias added to 1500):

```
error  = setpoint - measurement
output = kp * error + ki * integral(error * dt) + kd * d(error) / dt
```

You compute the error in the node (only you know the target -
a camera angle, a distance, etc.) and the class computes the
output. It returns a number but it does not drive the motor.

- **P** reacts to the error now. Alone it leaves a small steady offset.
- **I** sums the error over time and removes that offset.
- **D** reacts to how fast the error changes and damps oscillation.

## Units

The class is unit-agnostic, so each gain's unit is PWM / [error unit]. That is why the scale differ - `kp_surge` is
large because its error is in meters, `kp_yaw` is small because
its error is in degrees.

| axis  | error from                     | unit          | kp      |
|-------|--------------------------------|---------------|---------|
| yaw   | cam_h_angle_deg                | degrees       | PWM/deg |
| heave | cam_v_angle_deg                | degrees       | PWM/deg |
| surge | dist - TARGET_DISTANCE         | meters        | PWM/m   |
| depth | target_depth - current_depth   | meters (neg.) | PWM/m   |
| pitch | IMU orientation                | degrees       | PWM/deg |
| roll  | IMU orientation                | degrees       | PWM/deg |

Depth is negative (surface = 0, deeper = more negative), from
the depth estimator. Set `output_sign = -1` on an axis if a positive PWM bias makes the error grow there.

## Control rate

The node runs a ROS timer at a fixed rate and passes the same rate to
`AxisController(control_hz=...)`. The controller uses a fixed `dt = 1/control_hz`
for the I and D terms, so there is no need to measure time - but the timer
frequency and `control_hz` must be the same number.

## Usage

```python
from bluespark_autonomy.pid.axis_controller import AxisController

HZ = 10.0

# __init__:
self.axes = AxisController(control_hz = HZ)
self.create_timer(1.0 / HZ, self.control_loop)   # timer freq MUST equal control_hz
self.latest = None

# vision_callback: just store the detection
def vision_callback(self, msg):
    self.latest = msg

# control_loop: runs at fixed HZ
def control_loop(self):
    if self.latest is None:
        return
    target = ...  # pick target out of self.latest
    error_yaw = target.cam_h_angle_deg
    yaw_pwm = self.axes.update("yaw", error_yaw)   # no dt
    self.control_state.set_pwm("yaw", yaw_pwm)
```

Call `reset_all()` on state changes so a stale integral from the previous phase
doesn't kick the vehicle on the first new sample.

## AxisController API

| method                        | does                                             |
|-------------------------------|--------------------------------------------------|
| update(axis, error)       | one axis -> absolute PWM (1500 + bias, clamped)     |
| reset(axis) / reset_all()     | clears the integral and derivative history        |

`dt` is not a parameter - it is fixed to `1/control_hz` and applied inside update, because the ROS timer calls the loop at a constant rate.

## PID options

| arg            | meaning                                                   |
|----------------|-----------------------------------------------------------|
| kp, ki, kd     | gains                                                     |
| output_limits  | clamp on the PWM bias, e.g. (-150,150) -> 1350..1650      |
| integral_limit | anti-windup clamp on the I term; None = use output_limits |
| derivative_tau | D low-pass filter (s); 0 = off, ~0.1 when kd > 0          |
| output_sign    | +1 / -1, flip an axis with reversed motor direction       |

`output_limits` clamps the whole P+I+D sum; `integral_limit` clamps only the I
term before it's added (this is the anti-windup that stops the integral charging
up while the actuator is saturated).

## Starting gains

Placeholders - safe to run but not tuned. Tune in the pool.

| axis  | kp  | ki  | kd |
|-------|-----|-----|----|
| yaw   | 4.0 | 0.2 | 0  |
| heave | 4.0 | 0.5 | 0  |
| surge | 100 | 5.0 | 0  |
| depth | 200 | 10  | 20 |
| pitch | 4.0 | 0.2 | 0  |
| roll  | 4.0 | 0.2 | 0  |
