# Depth Hold Node

Depth controller for BlueSpark. Works on top of ArduSub's ALT_HOLD mode.
Reads the current depth, compares it with the target and adjusts the
heave RC channel to keep the robot at the right depth.

## How it works

`ALT_HOLD` keeps the robot at the depth where the mode was turned on.
To reach a specific depth, we push the heave channel up or down depending
on how far we are from the target depth. The further from the target, the stronger the push.

## Conventions

Depth in meters:
- surface = `0.0`
- below the surface = **negative** (e.g. `-0.3` means 30 cm under water)

PWM:
- `1500` = neutral, the autopilot controls the channel
- `> 1500` = up
- `< 1500` = down

## Topics and services

Subscribes to:
- `/current_depth` (`std_msgs/Float64`) — current depth from `depth_estimator_node`
- `/mavros/state` (`mavros_msgs/State`) — current FCU mode, the controller only works in `ALT_HOLD`

Calls service:
- `control/set_heave` (`bluespark_interfaces/SetRCOverride`) — sets the heave RC channel through `rc_override_node`

## Parameters

All parameters are currently hardcoded in `__init__`:
- `target_depth` = `-0.3` m - desired depth
- `deadband` = `0.1` m - band around the setpoint (avoid oscillating on sensor noise)
- `gain` = `200` [PWM/m] — how much PWM per meter of error
- `heave_up_max` = `1600` — upper PWM limit
- `heave_down_min` = `1400` — lower PWM limit

## Control logic

On every `/current_depth` message:

1. If the mode is not `ALT_HOLD`, do nothing.
2. Compute the error: `error = target_depth - current_depth`.
3. If `|error| < deadband`, send `1500` and let `ALT_HOLD` take over.
4. Otherwise compute the PWM:

```
pwm = 1500 + gain * error
```

Then limit it to `[heave_down_min, heave_up_max]` for safety before sending.

When the node shuts down, it sends `1500` so the autopilot regains the channel.

## Running

The node is launched with the rest of the system. To run it requires:
1. MAVROS up with a working barometer (`/mavros/imu/static_pressure` publishing).
2. `rc_override_node` from `bluespark_control` running (provides `control/set_heave`).
3. `depth_estimator_node` running and publishing on `/current_depth`.
4. Drone armed and switched to `ALT_HOLD` mode.

## TODO

- Get `target_depth` from the mission planner instead of hardcoding it.
- Move all parameters out of the code so they can be changed without editing the file.
