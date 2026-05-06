# Navigation Node (movement_node)

Main node used for the autonomous navigation of the AUV. The node processes data from the vision system and generates control signals for the motion controller to keep the target object centered in the frame.

---

## Running

It requires running within the package namespace:

```bash
ros2 run bluespark_navigation movement_node
```

---

## Dependencies

For full operation, the node requires the following components to be running:

| Node | Package | System Function |
| :--- | :--- | :--- |
| `vision_node` | `bluespark_vision` | Provides positions of objects detected by the camera. |
| `rc_override_node` | `bluespark_control` | Provides services to override PWM signals. |
| `vehicle_manager_node` | `bluespark_control` | Manages the transmission of generated signals to the thrusters. |
| `depth_hold_node` | `bluespark_navigation` | Maintains constant depth, offloading navigation control. |

---

## API Interface

### Subscribed Topics

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `/detected_objects` | `bluespark_interfaces/msg/DetectedObjectArray` | Input data containing object positions and angular error. |

### Service Clients

The node sends control commands (PWM range: 1100-1900) by calling the following services:

| Service Name | Service Type | Control Axis (DoF) |
| :--- | :--- | :--- |
| `control/set_pitch` | `bluespark_interfaces/srv/SetRCOverride` | Pitch angle |
| `control/set_roll` | `bluespark_interfaces/srv/SetRCOverride` | Roll angle |
| `control/set_yaw` | `bluespark_interfaces/srv/SetRCOverride` | Yaw angle (rotation) |
| `control/set_heave` | `bluespark_interfaces/srv/SetRCOverride` | Heave (vertical movement / depth) |
| `control/set_surge` | `bluespark_interfaces/srv/SetRCOverride` | Surge (forward / backward movement) |
| `control/set_sway` | `bluespark_interfaces/srv/SetRCOverride` | Sway (lateral / sideways movement) |

---

## Parameters

Currently, the node is in the MVP phase. Values such as the target distance (target_distance) and proportional controller gains (Kp) are hardcoded. These will be optimized and extracted into ROS 2 parameters in future iterations.

# Depth Estimator Node (depth_estimator_node)
Node used for calculating the current depth of AUV from raw pressure data (from baro30 sensor), using formula: h = (p_atm-p_current)/(d_liquid * g).

---

## Running
```bash
ros2 run bluespark_navigation depth_estimator_node
```

---

## Dependencies

This node requires following components to run properly:
| Node | Package | System function |
| :--- | :--- | :--- |
| `mavros_node` | `mavros` | Enables all the communication between pixhawk and ROS2 using sensor_msgs and std_msgs|

---

## API Interface

### Subscribed topics

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `mavros/imu/static_pressure` | `FluidPressure` | Raw data from pressure sensor to calculate altitude |

### Created publishers

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `/current_depth` | `Float64` | Calculated altitude (negative under the water surface) |

Values such as density of water and g are hardcoded, because the author doesn't expect to test the robot in another liquid or on another planet than earth.

---

# Depth Hold Node

Depth controller for BlueSpark. Works on top of ArduSub's `ALT_HOLD` mode.
Reads the current depth, compares it with the target and adjusts the
heave RC channel to keep the robot at the right depth.

`ALT_HOLD` keeps the robot at the depth where the mode was turned on.
To reach a specific depth, we push the heave channel up or down depending
on how far we are from the target depth. The further from the target, the stronger the push.

---

## Running

```bash
ros2 run bluespark_navigation depth_hold_node
```

---

## Dependencies

This node requires the following components to run properly:

| Node | Package | System Function |
| :--- | :--- | :--- |
| `mavros_node` | `mavros` | Provides the FCU mode (`ALT_HOLD`) over `/mavros/state`. |
| `depth_estimator_node` | `bluespark_navigation` | Provides the current depth on `/current_depth`. |
| `rc_override_node` | `bluespark_control` | Provides the service to override the heave channel. |

---

## API Interface

### Subscribed Topics

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `/current_depth` | `std_msgs/Float64` | Current depth from `depth_estimator_node`. |
| `/mavros/state` | `mavros_msgs/State` | Current FCU mode. Controller only acts when mode is `ALT_HOLD`. |

### Service Clients

| Service Name | Service Type | Control Axis (DoF) |
| :--- | :--- | :--- |
| `control/set_heave` | `bluespark_interfaces/srv/SetRCOverride` | Heave (vertical movement / depth) |

---

## Conventions

Depth in meters:
- surface = `0.0`
- below the surface = **negative** (e.g. `-0.3` means 30 cm under water)

PWM:
- `1500` = neutral, the autopilot controls the channel
- `> 1500` = up
- `< 1500` = down

---

## Control logic

On every `/current_depth` message:

1. If the mode is not `ALT_HOLD`, do nothing.
2. Compute the error: `error = target_depth - current_depth`.
3. If `|error| < deadband`, send `1500` and let `ALT_HOLD` take over.
4. Otherwise compute the PWM:

```
pwm = 1500 + gain * error
```

5. Limit it to `[heave_down_min, heave_up_max]` for safety before sending.

When the node shuts down, it sends `1500` so the autopilot regains the channel.

---

## Parameters

All parameters are currently hardcoded in `__init__`:

| Name | Value | Description |
| :--- | :--- | :--- |
| `target_depth` | `-0.3` m | Desired depth (negative = below surface). |
| `deadband` | `0.1` m | Band around the setpoint, avoids oscillating on sensor noise. |
| `gain` | `200` [PWM/m] | How much PWM per meter of error. |
| `heave_up_max` | `1600` | Upper PWM limit. |
| `heave_down_min` | `1400` | Lower PWM limit. |

These will be moved to ROS 2 parameters in future iterations.

---

## TODO

- Get `target_depth` from the mission planner instead of hardcoding it.
- Move all parameters out of the code so they can be changed without editing the file.
