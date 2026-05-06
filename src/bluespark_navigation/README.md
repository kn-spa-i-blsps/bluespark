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