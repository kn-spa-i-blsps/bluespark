# StateListenerNode (vehicle_manager_node)

Node responsible for gathering information of current state of robot: connected/disconnected, armed/disarmed  and what mode it is in.

---

## Running

It requires running within the package namespace:

```bash
ros2 run bluespark_control vehicle_manager_node
```

---

## Dependencies

For correct operation, the node requires the such components to be running:

| Node | Package | System Function |
| :--- | :--- | :--- |
| `mavros_node` | `mavros` | Enables all the communication between pixhawk and ROS2 using mavros_msgs (State). |

The node uses mavros_msgs/msg/State to work properly.

---

## API Interface

### Topic listeners

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
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

| Node | Package | System Function |
| :--- | :--- | :--- |
| `mavros_node` | `mavros` | Enables all the communication between pixhawk and ROS2 using mavros_msgs.|

The node also requires mavros CommandBool and SetBool services.

---

## API Interface

### Created services

| Service Name | Service Type | Description |
| :--- | :--- | :--- |
| `/manager/set_arming` | `std_srvs/srv/SetBool` | Arming or disarming, takes True or False. |

### Created clients

| Service Name | Service Type | Description |
| :--- | :--- | :--- |
| `/mavros/cmd/arming` | `mavros_srvs/srv/CommandBool` | Calling mavros service to arm or disarm the robot. |

---

# SetModeNode (vehicle_manager_node)

Node responsible for setting the mode of a robot, once there is connection between ros2 and pihawk.

---

## Running

It requires running within the package namespace:

```bash
ros2 run bluespark_control vehicle_manager_node
```

---

### Dependencies

For correct operation, the node requires the such components to be running:

| Node | Package | System Function |
| :--- | :--- | :--- |
| `mavros_node` | `mavros` | Enables all the communication between pixhawk and ROS2 using mavros_msgs. |

The node also requires mavros CommandBool and SetMode serives.

---

## API Interface

### Created services

| Service Name | Service Type | Description |
| :--- | :--- | :--- |
| `/manager/set_mode` | `mavros_srvs/srv/SetMode` | Setting given in capital letters mode, default: GUIDED. |

### Created clients

| Service Name | Service Type | Description |
| :--- | :--- | :--- |
| `/mavros/set_mode` | `mavros_srvs/srv/SetMode` | Calling mavros service set given mode of the robot. |

---

# RCOverriteNode (rc_override_node)

Main node responsible for sending PWM to engines (in range 1100-1900). It holds array which consist all the RC channels, and sends it further to the pixahwk.
Note that the particular DOF callback only modify the array, not actually send the message to pixhawk.
The node in fact emulates the controller in hand of a operator.

---

## Running

It requires running within the package namespace:

```bash
ros2 run bluespark_control vehicle_manager_node
```

---

## Dependencies

For correct operation, the node requires the such components to be running:

| Node | Package | System Function |
| :--- | :--- | :--- |
| `mavros_node` | `mavros` | Enables all the communication between pixhawk and ROS2 using mavros_msgs. |
| `ArmingNode` | `bluespark_control` | Manages the arming and disarming the robot. The node has to be called before RCOverride is used. |

---

## API Interface

### Created publishers

| Topic Name | Message Type | Description |
| :--- | :--- | :--- |
| `/mavros/rc/override` | `mavros_msgs/OverrideRCIn` | Input of modified RC channels. |

### Launched services

The node allows to send PWM to the robot (range: 1100-1900) by calling the following services:

| Service Name | Service Type | Control Axis (DoF) |
| :--- | :--- | :--- |
| `control/set_pitch` | `bluespark_interfaces/srv/SetRCOverride` | Pitch angle |
| `control/set_roll` | `bluespark_interfaces/srv/SetRCOverride` | Roll angle |
| `control/set_yaw` | `bluespark_interfaces/srv/SetRCOverride` | Yaw angle (rotation) |
| `control/set_heave` | `bluespark_interfaces/srv/SetRCOverride` | Heave (vertical movement / depth) |
| `control/set_surge` | `bluespark_interfaces/srv/SetRCOverride` | Surge (forward / backward movement) |
| `control/set_sway` | `bluespark_interfaces/srv/SetRCOverride` | Sway (lateral / sideways movement) |

---



