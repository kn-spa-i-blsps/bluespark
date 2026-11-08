# bluespark_control

Low-level control layer between the autonomy (behaviour tree) and MAVROS. Two executables:

- `vehicle_manager_node` - arming / disarming and flight mode (StateListener, ArmingNode, SetModeNode in one process)
- `rc_override_node` - sends PWM to the thrusters through RC override

On the vehicle both are started by `bluespark_bringup/governer.launch.py`

---

# StateListenerNode (vehicle_manager_node)

Node responsible for gathering information of current state of robot: connected/disconnected, armed/disarmed and what mode it is in.

---

## Running

It requires running within the package namespace:

```bash
ros2 run bluespark_control vehicle_manager_node
```

---

## Dependencies

For correct operation, the node requires the such components to be running:

| Node          | Package  | System Function                                                                   |
| :------------ | :------- | :-------------------------------------------------------------------------------- |
| `mavros_node` | `mavros` | Enables all the communication between pixhawk and ROS2 using mavros_msgs (State). |

The node uses mavros_msgs/msg/State to work properly.

---

## API Interface

### Topic listeners

| Topic Name      | Message Type        | Description                                         |
| :-------------- | :------------------ | :-------------------------------------------------- |
| `/mavros/state` | `mavros_msgs/State` | Basic information about current state of the robot. |

---

# ArmingNode (vehicle_manager_node)

Node responsible for arming and disarming the robot.

---

## Running

It requires running within the package namespace:

```bash
ros2 run bluespark_control vehicle_manager_node
```

---

## Dependencies

For correct operation, the node requires the such components to be running:

| Node          | Package  | System Function                                                           |
| :------------ | :------- | :------------------------------------------------------------------------ |
| `mavros_node` | `mavros` | Enables all the communication between pixhawk and ROS2 using mavros_msgs. |

The node also requires the mavros `/mavros/cmd/arming` service (`mavros_msgs/srv/CommandBool`).

---

## API Interface

### Created services

| Service Name          | Service Type           | Description                               |
| :-------------------- | :--------------------- | :---------------------------------------- |
| `/manager/set_arming` | `std_srvs/srv/SetBool` | Arming or disarming, takes True or False. |

> **Note:** the response is always `success: True` once the command is passed to mavros. It does **not** mean the robot actually armed (e.g. a failed pre-arm check). Check `armed` in `/mavros/state`.

### Created clients

| Service Name         | Service Type                  | Description                                        |
| :------------------- | :---------------------------- | :------------------------------------------------- |
| `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | Calling mavros service to arm or disarm the robot. |

---

# SetModeNode (vehicle_manager_node)

Node responsible for setting the mode of a robot, once there is connection between ros2 and pixhawk.

---

## Running

It requires running within the package namespace:

```bash
ros2 run bluespark_control vehicle_manager_node
```

---

### Dependencies

For correct operation, the node requires the such components to be running:

| Node          | Package  | System Function                                                           |
| :------------ | :------- | :------------------------------------------------------------------------ |
| `mavros_node` | `mavros` | Enables all the communication between pixhawk and ROS2 using mavros_msgs. |

The node also requires the mavros `/mavros/set_mode` service (`mavros_msgs/srv/SetMode`).

---

## API Interface

### Created services

| Service Name        | Service Type              | Description                                                                           |
| :------------------ | :------------------------ | :------------------------------------------------------------------------------------ |
| `/manager/set_mode` | `mavros_msgs/srv/SetMode` | Sets the mode given in `custom_mode`, in capital letters (e.g. `MANUAL`, `ALT_HOLD`). |

> **Note:** the response is always `mode_sent: True` once the command is passed to mavros. It does **not** mean the mode was actually set. Check `mode` in `/mavros/state`.

### Created clients

| Service Name       | Service Type              | Description                                         |
| :----------------- | :------------------------ | :-------------------------------------------------- |
| `/mavros/set_mode` | `mavros_msgs/srv/SetMode` | Calling mavros service set given mode of the robot. |

---

# RCOverrideNode (rc_override_node)

Main node responsible for sending PWM to engines (expected range 1100-1900, not validated yet). It holds array which consist all the RC channels, and sends it further to the pixhawk.
Note that the particular DOF callback only modify the array, not actually send the message to pixhawk.
The node in fact emulates the controller in hand of a operator.

---

## Running

It requires running within the package namespace:

```bash
ros2 run bluespark_control rc_override_node
```

---

## Dependencies

For correct operation, the node requires the such components to be running:

| Node          | Package             | System Function                                                                                  |
| :------------ | :------------------ | :----------------------------------------------------------------------------------------------- |
| `mavros_node` | `mavros`            | Enables all the communication between pixhawk and ROS2 using mavros_msgs.                        |
| `ArmingNode`  | `bluespark_control` | Manages the arming and disarming the robot. The node has to be called before RCOverride is used. |

---

## API Interface

### Created publishers

| Topic Name            | Message Type               | Description                    |
| :-------------------- | :------------------------- | :----------------------------- |
| `/mavros/rc/override` | `mavros_msgs/OverrideRCIn` | Input of modified RC channels. |

### Launched services

The node allows to send PWM to the robot (range: 1100-1900) by calling the following services:

| Service Name        | Service Type                             | Control Axis (DoF)                  |
| :------------------ | :--------------------------------------- | :---------------------------------- |
| `control/set_pitch` | `bluespark_interfaces/srv/SetRCOverride` | Pitch angle                         |
| `control/set_roll`  | `bluespark_interfaces/srv/SetRCOverride` | Roll angle                          |
| `control/set_yaw`   | `bluespark_interfaces/srv/SetRCOverride` | Yaw angle (rotation)                |
| `control/set_heave` | `bluespark_interfaces/srv/SetRCOverride` | Heave (vertical movement / depth)   |
| `control/set_surge` | `bluespark_interfaces/srv/SetRCOverride` | Surge (forward / backward movement) |
| `control/set_sway`  | `bluespark_interfaces/srv/SetRCOverride` | Sway (lateral / sideways movement)  |

A service call only changes the value in the array. The whole array is published to `/mavros/rc/override` every 0.1 s (10 Hz), so a new value reaches the robot within 100 ms. The last value is held until it is changed.

### RC channels

Mapping follows the ArduSub default `RCMAP_*` parameters:

| Channel | Axis             |
| :------ | :--------------- |
| 1       | pitch            |
| 2       | roll             |
| 3       | heave (throttle) |
| 4       | yaw              |
| 5       | surge (forward)  |
| 6       | sway (lateral)   |
| 7-18    | not used         |

PWM values:

| Value               | Meaning                                                         |
| :------------------ | :-------------------------------------------------------------- |
| `0`                 | channel released (no override). All channels start at 0.        |
| `1500`              | neutral / stop                                                  |
| `> 1500` / `< 1500` | move in one / the other direction, further from 1500 = stronger |

Expected range is 1100-1900. The direction for each axis (e.g. whether heave > 1500 is up or down) should be confirmed on the bench.

---
