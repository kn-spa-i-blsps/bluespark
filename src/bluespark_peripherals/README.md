# bluespark_peripherals

> **Status: work in progress, never tested on hardware.**
> This package was written in a hurry before RoboSub and it needs some more love
> before it can be trusted in a mission. It is kept because payload handling is
> something we will need anyway. See the TODO list below.

Controls the mission payload mechanisms (two ball drops and a torpedo) with servos
connected to a PCA9685 PWM board over I2C.

---

## How it works

```
behaviour tree (TriggerPayload)  ──▶  /payload/action (std_msgs/Int32)  ──▶  servo_node  ──▶  PCA9685  ──▶  servo
```

- `servo_node` listens on `/payload/action` and runs the requested action.
- Every action does the same: servo goes to 90°, waits 0.5 s, goes back to 0°.
- If the Adafruit libraries or the I2C bus are not available (e.g. on a laptop),
  the driver runs in **simulation mode**: it starts, but every action reports a failure.

| Value | Action                     | PCA9685 channel |
| :---- | :------------------------- | :-------------- |
| `0`   | `DROP_BALL_1`              | 12              |
| `1`   | `DROP_BALL_2`              | 13              |
| `2`   | `FIRE_TORPEDO`             | 14              |
| `99`  | `RESET` (all servos to 0°) | 12, 13, 14      |

I2C address `0x40`, 50 Hz, pulse range 800-2200 µs (see `servo/driver.py`).

---

## Running

`servo_node` is **not started by any launch file yet**. To try it by hand:

```bash
ros2 run bluespark_peripherals servo_node

# in another shell
ros2 topic pub --once /payload/action std_msgs/msg/Int32 "{data: 0}"
```

From the behaviour tree use `TriggerPayload` from `bluespark_autonomy/behaviours`.

---

## TODO

- Test on real hardware and tune the servo angles (now hardcoded 90° / 0°).
- Decide which Raspberry Pi the PCA9685 is connected to and start `servo_node`
  from that Pi's launch file.
- Commands can be lost silently: `TriggerPayload` publishes a single message and
  reports SUCCESS after 5 s even if nobody received it. Switch to a service that
  returns whether the action worked.
- Call `reset()` on startup, so the servos start in a known position.
- Move channels, I2C address and angles to ROS parameters.
- Use the ROS logger instead of `print()` in `servo/driver.py`.
- Remove the unused `DetectionResult` dataclass from `utils/types.py`.
- `package.xml`: depends on `std_srvs`, but the node uses `std_msgs`. Add a description.
