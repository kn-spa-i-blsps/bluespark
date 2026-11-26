# Bench testing with `./bs`

`./bs` runs BlueSpark's control stack on the **governer Pi** for bench tests:
start it, check that everything works, run missions, control thrusters by hand,
stop it. Every session is logged automatically.

Everything is run on the governer Pi, in the repo folder (over SSH is fine).

---

## Safety first

- `./bs test`, `./bs arm`, `./bs pwm` and missions **spin the thrusters**.
  Props off or guarded, hands away, kill switch within reach.
- **Emergency:** `./bs disarm` (disarm + all thrusters to stop). Works at any time
  while the stack runs, from any terminal.
- A mission is **always** stopped safely at the end: disarm + all thrusters to
  1500 (stop), also after Ctrl+C or when the mission crashes.

---

## Before the first session (once per Pi)

1. The repo is on the Pi, on the right branch:
   ```bash
   cd ~/bluespark            # or wherever the repo is
   git checkout 18-fix-bringups
   git pull
   ```
2. Docker works without `sudo` (`docker ps` prints a table, not "permission denied").
3. The Pixhawk is connected by USB (`ls /dev/ttyACM*` shows `/dev/ttyACM0`).
4. Nothing else is using the Pixhawk: if the production stack runs on this Pi,
   stop it first: `docker compose -f docker-compose.prod.yml down`.

---

## A normal session

### 1. Start

```
$ ./bs start
Starting the stack. Session logs: logs/2026-11-24_01-11-16/
Building the workspace (colcon)...
[ OK ] Workspace built
[ OK ] mavros started, waiting for the Pixhawk...
[ OK ] Pixhawk connected
[ OK ] Control nodes ready

READY.  Next: ./bs test   or   ./bs mission <name>   (./bs missions lists them)
```

The first start after a `git pull` takes longer (it builds the code). If it stops
with `[FAIL]`, it says why; see [Troubleshooting](#troubleshooting).

### 2. Test everything

```
$ ./bs test
Bench test: thrusters WILL spin (1600 for 2 s per axis). Ctrl+C = stop + disarm.

 1. Stack nodes running          OK   mavros, rc_override_node, vehicle_manager_node
 2. Pixhawk connected            OK
 3. Mode MANUAL                  OK   mode MANUAL
 4. Arm                          OK   armed
 5. surge 1600 for 2 s           OK   1600 seen on /mavros/rc/override
 6. sway 1600 for 2 s            OK   1600 seen on /mavros/rc/override
 7. heave 1600 for 2 s           OK   1600 seen on /mavros/rc/override
 8. yaw 1600 for 2 s             OK   1600 seen on /mavros/rc/override
 9. roll 1600 for 2 s            OK   1600 seen on /mavros/rc/override
10. pitch 1600 for 2 s           OK   1600 seen on /mavros/rc/override
11. Mode ALT_HOLD                OK   mode ALT_HOLD
12. Mode MANUAL                  OK   mode MANUAL
    Disarm + thrusters 1500      OK

Result: ALL OK  (12/12 OK)
```

One axis at a time, so you can **watch which thrusters move** for each axis.
"seen on /mavros/rc/override" means our control node really sent the value to
mavros, which passes it to the Pixhawk. It does not prove the Pixhawk moved the
thrusters: that is what your eyes are for.
Gentler or longer: `./bs test --step 50 --seconds 3`.

### 3. Work

Run a mission (see [Test missions](#test-missions)):

```bash
./bs missions                  # what is there
./bs mission forwardTurnBack   # runs it once; Ctrl+C stops it safely
```

When it ends you get a summary:

```
============================================================
 forwardTurnBack: OK   (12.5 s)
 safety stop: OK: disarmed; thrusters neutral (1500)
 logs: logs/2026-11-24_01-30-54/01_01-31-07_forwardTurnBack/
============================================================
```

A mission runs **once** and ends. If any step fails, it stops right there and
the result says `FAILED`. Ctrl+C stops it; Ctrl+C twice kills it immediately.
In every case the vehicle is disarmed and the thrusters set to 1500 at the end.

Or control it by hand:

```bash
./bs mode MANUAL
./bs arm
./bs pwm surge 1600           # 1100-1900, 1500 = stop
./bs pwm surge 1500
./bs althold                  # hold depth (ALT_HOLD); back: ./bs mode MANUAL
./bs neutral                  # all thrusters 1500
./bs disarm
./bs status                   # what is going on now
```

Axes: `surge` (forward/back), `sway` (left/right), `heave` (up/down), `yaw`,
`roll`, `pitch`.

### 4. Stop

```
$ ./bs stop
Disarming...
[ OK ] disarmed
[ OK ] thrusters neutral (1500)
Stopping the stack...
[ OK ] Stopped
Session logs: logs/2026-11-24_01-11-16/
```

---

## Simulation (`./bs sim`)

The same session, but against the simulation instead of the Pixhawk: ArduSub
SITL (the Pixhawk firmware compiled for Linux) drives the BlueROV2 in Gazebo.
Runs on a laptop; nothing is plugged in. Everything below the start works the
same: `./bs test`, `./bs mission <name>`, `./bs pwm ...`, `./bs logs`, `./bs stop`.

1. Start the simulation by hand, in the `bs_sim` container (repo
   `bluespark-simulation`, `./start_docker.sh`; see its README):
   - terminal 1: `bash scripts/run_gazebo.sh`
   - terminal 2: `bash scripts/run_ardusub.sh`, wait for the `MANUAL>` prompt
2. Here: `./bs sim`

```
$ ./bs sim
[ OK ] Simulation is up: Gazebo + ArduSub SITL in bs_sim
Starting the stack (SIMULATION, ArduSub SITL). Session logs: logs/...
[ OK ] mavros started, waiting for the ArduSub SITL...
[ OK ] ArduSub SITL connected
[ OK ] Control nodes ready
```

- Before starting, `./bs sim` checks that `bs_sim`, Gazebo and ArduSub/MAVProxy
  run and that UDP port 14550 (where MAVProxy sends MAVLink) is free, and says
  what to start if not.
- `./bs restart` stays in simulation; `session_info.txt` has `mode sim`.
- Simulation sessions are marked so they are never mistaken for real tests:
  the folder ends with `_sim` (`logs/2026-11-26_22-40-00_sim/`), and
  `./bs logs` / `./bs status` show `[SIM]`.
- `./bs stop` stops only the stack; Gazebo and ArduSub keep running.
- Restarted ArduSub? Then also `./bs restart`.

---

## Test missions

All three use the **Pixhawk's own depth hold (ALT_HOLD)**: they switch to ALT_HOLD,
check that the Pixhawk really is in ALT_HOLD (if not, e.g. no depth sensor, the
mission fails before the thrusters move), arm, move, stop, disarm.

| Mission | What it does | Parameters (default) |
| :-- | :-- | :-- |
| `forwardTurnBack` | Forward, stop, turn 180°, stop, forward back to the start | `surge_pwm` (1600), `forward_sec` (5.0), `yaw_pwm` (1600), `turn_sec` (4.0), `pause_sec` (2.0) |
| `circleFacingCenter` | One circle **sideways**, the front always facing the center | `sway_pwm` (1600), `yaw_pwm` (1400), `circle_sec` (20.0) |
| `circleForward` | One circle **front first** (facing where it goes) | `surge_pwm` (1600), `yaw_pwm` (1600), `circle_sec` (20.0) |

Moves are **time-based** (a fixed PWM for a fixed time), so they need tuning at
the pool. Change a parameter without touching the code:

```bash
./bs mission forwardTurnBack --ros-args -p turn_sec:=3.5            # turned too far
./bs mission circleForward --ros-args -p circle_sec:=25.0           # circle not closed
./bs mission forwardTurnBack --ros-args -p forward_sec:=3.0 -p pause_sec:=1.0
```

The values used are written at the top of the mission's log (`Parameters: ...`)
and in `info.txt` (`command`).

Which way 1600 turns or moves the vehicle is **not confirmed yet**. If it goes the
wrong way, use the other side of 1500 (1400 instead of 1600). For
`circleFacingCenter`, `sway_pwm` and `yaw_pwm` must be on opposite sides of 1500;
if the front drifts away from the center, swap `yaw_pwm` to the other side.

---

## All commands

| Command | What it does |
| :-- | :-- |
| `./bs start` | Start the stack (no mission). Shows each step, waits until ready. |
| `./bs sim` | The same, against the simulation (ArduSub SITL in Gazebo), see [Simulation](#simulation-bs-sim). |
| `./bs test` | Check the whole chain step by step. **Thrusters spin.** Options: `--step 50`, `--seconds 3` |
| `./bs mission <name>` | Run a mission once, with full logs. Ctrl+C = stop it safely. Parameters: `--ros-args -p name:=value` |
| `./bs stop` | Disarm and stop everything. |
| `./bs disarm` | **Emergency**: disarm + all thrusters 1500. |
| `./bs status` | Pixhawk connected? Armed? Mode, thrusters, battery, nodes, last events. |
| `./bs arm` | Arm. The answer comes from the Pixhawk. |
| `./bs althold` | Mode ALT_HOLD (hold depth). |
| `./bs mode <MODE>` | Any mode: `MANUAL`, `STABILIZE`, `ALT_HOLD`, ... |
| `./bs pwm <axis> <value>` | One axis, 1100-1900, 1500 = stop. |
| `./bs neutral` | All thrusters 1500. |
| `./bs missions` | List the missions. |
| `./bs logs` | Mission runs of the session and their result. |
| `./bs logs -f` | Follow the session log live (Ctrl+C to quit). |
| `./bs restart` | Stop + start (e.g. after `git pull`), in the same mode (`sim` stays `sim`). |
| `./bs shell` | A shell inside the container: plan B, see below. |
| `./bs help` | This list, short. |

---

## Logs

Everything from `./bs start` to `./bs stop` is one **session**, in `logs/`
(a simulation session, `./bs sim`, has `_sim` at the end of the folder name):

```
logs/2026-11-24_01-11-16/
├── session_info.txt                  when, which Pi, which git commit, changed files
├── timeline.log                      everything that happened, in order
└── 01_01-31-07_forwardTurnBack/      one folder per mission run
    ├── info.txt                      result, times, safety stop, git commit
    ├── console.log                   everything the mission printed
    ├── bag/                          recording of all topics (no camera images)
    ├── bag_record.log
    ├── ros_log/                      ROS log files of the mission
    └── git_diff.patch                only if the code had uncommitted changes
```

### `timeline.log`: what happened, in order

The most useful file. Every line is one event, from all programs together:

```
01:11:36.802  =====  START bench test (./bs test)  =====
01:11:39.141  STATE  ARMED
01:11:39.432  RC     surge released -> 1600
01:11:41.532  RC     surge 1600 -> 1500, sway released -> 1600
01:11:52.562  STATE  mode MANUAL -> ALT_HOLD
01:11:54.637  STATE  DISARMED
01:11:54.638  =====  END bench test: 12/12 OK  =====
01:31:10.330  =====  START mission 01 forwardTurnBack (git 4e95764)  =====
01:31:10.746  INFO   [forward_turn_back_node] Parameters: surge_pwm=1600, forward_sec=1.5, ...
01:31:15.754  STATE  mode MANUAL -> ALT_HOLD
01:31:15.825  INFO   [forward_turn_back_node] [tree] Confirm ALT_HOLD: RUNNING -> SUCCESS (Pixhawk in ALT_HOLD)
01:31:16.753  STATE  ARMED
01:31:17.527  INFO   [forward_turn_back_node] [tree] Swim Forward: RUNNING -> SUCCESS
01:31:18.151  RC     yaw 1500 -> 1600
01:31:19.226  INFO   [forward_turn_back_node] [tree] Turn 180: RUNNING -> SUCCESS
01:31:22.869  STATE  DISARMED
01:31:22.877  =====  END mission 01 forwardTurnBack: OK (12.5 s)  =====
```

When a mission fails, the line with `FAILURE` says which step and why:

```
01:30:29.384  ERROR  [forward_turn_back_node] [tree] Confirm ALT_HOLD: RUNNING -> FAILURE (Pixhawk still in MANUAL after 5 s, not ALT_HOLD)
```

| Kind | Meaning |
| :-- | :-- |
| `=====` | Start / end of the session, a test or a mission |
| `STATE` | Pixhawk: connected / disconnected, armed / disarmed, mode |
| `RC` | Thruster commands per axis (only changes). `released` = no command |
| `INFO` `WARN` `ERROR` | A message of a program, `[name]` = which one. `[tree]` lines = mission steps. Messages from the Pixhawk itself (e.g. why it refused to arm) appear here too, from `mavros` |

Follow it live in a second terminal: `./bs logs -f`.

### Mission results (`info.txt`, `./bs logs`)

| Result | Meaning |
| :-- | :-- |
| `OK` | All steps done |
| `FAILED: a step of the mission failed` | A step failed, the mission stopped there. Which one and why: the `FAILURE` line in `timeline.log` |
| `FAILED: setup (services missing)` | The mission could not start: a part of the stack is not running. `./bs status`, `./bs restart` |
| `FAILED (exit code N, see console.log)` | The mission program crashed: the error is in `console.log` |
| `INTERRUPTED (Ctrl+C)` | Stopped with Ctrl+C |
| `KILLED` | Killed (e.g. Ctrl+C twice) |

`safe_stop` in `info.txt` says whether the disarm at the end was confirmed.

### Copy logs to a laptop

```bash
scp -r <user>@<pi-address>:<repo>/logs/2026-11-24_01-11-16 .
```

The `bag/` folder opens in PlotJuggler or Foxglove (`.mcap` file), or replays with
`ros2 bag play bag/`.

---

## Troubleshooting

| You see | Do this |
| :-- | :-- |
| `No Pixhawk. Check the USB cable and power` | Check the cable and that the Pixhawk is powered. `ls /dev/ttyACM*` must show `/dev/ttyACM0`. Then `./bs stop && ./bs start`. |
| `No MAVLink from ArduSub SITL` (`./bs sim`) | In `bs_sim`, `run_ardusub.sh` must be at the `MANUAL>` prompt. Then `./bs stop && ./bs sim`. |
| `UDP port 14550 is already in use` (`./bs sim`) | Another mavros listens there, e.g. a `ros2 launch ... governer.launch.py` in the dev container. Stop it. |
| `The code does not build` | Someone pushed broken code, or you have local changes: `git status`, `git pull`. Full log: `docker logs bluespark_bench`. |
| `The stack is not running` | `./bs start` |
| `Already running` after `./bs start` | Fine, it is already up. To start fresh: `./bs restart`. |
| Test step `Arm` FAIL | The Pixhawk refused. The reason is in the log: `./bs logs -f` or `timeline.log` (lines from `mavros`, e.g. PreArm ...). |
| Test step `Mode ALT_HOLD` FAIL, or a mission fails at `Confirm ALT_HOLD` | The Pixhawk refuses ALT_HOLD: it needs a working depth sensor. Check it in QGroundControl. |
| The vehicle turns / moves the wrong way in a mission | Use the other side of 1500 for that parameter, see [Test missions](#test-missions). |
| `... NOT seen on /mavros/rc/override` | The thruster commands do not reach the Pixhawk: `./bs status`, then `./bs restart`. |
| `Stack nodes running` FAIL | A program of the stack died: `./bs restart`. |
| A mission must be stopped now | Ctrl+C. Again Ctrl+C if it does not stop. Then check `./bs status`. |
| Thrusters keep spinning after something went wrong | `./bs disarm`. If that fails: kill switch. |
| `permission denied` from docker | Your user is not in the `docker` group: `sudo usermod -aG docker $USER`, log out and in. |
| Cannot delete files in `logs/` | Some are created by the container: `sudo rm -rf logs/<session>`. |

---

## Plan B: by hand

If `./bs` itself is in the way, everything still works by hand inside the container:

```bash
./bs shell                    # or: docker exec -it bluespark_bench bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 topic echo /mavros/state --once
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"   # disarm
ros2 service call /control/set_surge bluespark_interfaces/srv/SetRCOverride "{pwm_value: 1600}"
ros2 run bluespark_bringup vehicle status        # the same tools ./bs uses
ros2 run bluespark_bringup mission_runner forwardTurnBack
```

Without `./bs` at all: `docker compose -f docker-compose.bench.yml up -d` /
`down` (no session logs then).

---

## What `./bs` is not

- **Not production.** It uses `docker-compose.bench.yml` (stack without a mission).
  The production setup (`docker-compose.prod.yml`, `run_governer.sh`) is untouched.
- **No cameras.** Governer only.
