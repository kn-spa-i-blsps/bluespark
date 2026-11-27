# bluespark_control

Low-level control layer between the autonomy (behaviour tree) and MAVROS:

- `rc_override_node` - sends PWM to the thrusters through RC override

Arming, disarming and flight modes go straight to mavros (`/mavros/cmd/arming`,
`/mavros/set_mode`): the tree (`ArmRobot`, `SetFlightMode`), `./bs` / `vehicle` and
`run_governer.sh` all call it directly and check the answer.

On the vehicle `rc_override_node` is started by `bluespark_bringup/governer.launch.py`,
together with mavros and without waiting for it: the overrides reach the Pixhawk as
soon as mavros subscribes. The launch respawns it if it dies. Check that the whole stack is up with
`ros2 run bluespark_bringup vehicle ready`.

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
| Arming        | `mavros`            | The vehicle must be armed (`/mavros/cmd/arming`) before the overrides move the thrusters.        |

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
