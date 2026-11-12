# TODO

Known issues and planned work on `main` (reviewed 2026-11-12).
Tick an item (`- [x]`) in the PR that fixes it. Later this list can be moved to GitHub Issues.
Short `# TODO` comments in the code can stay next to the line they are about, but every one of
them should also be listed here.

---

## Safety and blockers (do first)

- [ ] **Prod on the governer is broken.** `governer.launch.py` defaults to
      `autonomy_executable:=hardcoded_gate`, which was deleted (ba66daa). The launch fails when the
      mission starts and `run_governer.sh` keeps restarting it. Change the default (e.g. `approacherTest`).
      Remember that prod then **arms and runs that mission on every start**, also after every reboot via systemd.
- [ ] **Dead man switch.** `rc_override_node` repeats the last PWM forever, so if the mission
      crashes mid-move the robot keeps going. Publish a heartbeat from the behaviour tree
      (post-tick handler) and set all axes to 1500 after ~1 s without it. Arm the watchdog only
      after the first heartbeat, so manual CLI service calls still work.
- [ ] **Ctrl+C does not stop thrusters or disarm.** rclpy handles SIGINT and shuts down the
      context before `finally`, so the cleanup never reaches mavros. Use
      `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` and handle SIGINT/SIGTERM
      ourselves in every node (control nodes, missions).
      In prod `run_governer.sh` disarms before stopping the nodes; bench/manual runs have no safety net.
- [ ] **Mission crash is not noticed by the supervisor.** Add `on_exit=Shutdown()` to the autonomy
      node in `governer.launch.py`. Must go together with changing `restart:` for the governer in
      `docker-compose.prod.yml`, otherwise a cleanly finished mission restarts forever.
- [ ] **Mission starts after a fixed 12 s, not after the Pixhawk is connected.** If mavros is not
      connected yet, `ArmRobot` fails and the mission ends without doing anything. Wait for
      `state/connected` before arming.
- [ ] **`/manager/set_arming` and `/manager/set_mode` always report success**, before mavros answers.
      The tree is fine (`ArmRobot` / `SetFlightMode` call mavros directly and check the result),
      but `run_governer.sh` disarms through `/manager/...`, so a failed disarm goes unnoticed.
- [ ] Clamp PWM to 1100-1900 in the `control/set_*` services (0 = release stays allowed).
- [ ] Check ArduSub failsafe parameters in QGC (pilot input / GCS loss, `RC_OVERRIDE_TIME`).

---

## bluespark_autonomy

- [ ] `DepthControl` feeds a depth error in meters into the `heave` PID (tuned for camera angles in
      degrees), so it barely reacts. Use the `depth` PID: `update("heave", depth_error)` ->
      `update("depth", depth_error)` in `actions.py` (there is a FIXME on that line).
- [ ] `DepthControl` reaches the depth and returns SUCCESS; nothing holds depth during other actions
      except ArduSub ALT_HOLD. Consider a depth-hold behaviour running in `Parallel`.
- [ ] No way to test a mission without a Pixhawk: `approacherTest` stops at `Set ALT_HOLD`.
      Add a `skip_fcu` parameter (skip mode + arm), which `DEV_TEST.md` already assumes.
- [ ] `behaviours/payload.py` (`TriggerPayload`) imports `bluespark_peripherals`, which was removed
      (520869f). Nothing imports it now, but any import fails. Delete it or move `PayloadAction`
      into autonomy until payload comes back.
- [ ] `autonomy_node` entry point in `setup.py` points to a file with only a comment (no `main`), so
      `ros2 run bluespark_autonomy autonomy_node` fails. Remove the entry point or implement it
      (idea in the file: one node that starts the mission given as an argument).
- [ ] `MoveRC` (`hardcoded_actions.py`) sends all 6 axes (default 1500), so it overrides heave set by
      other behaviours. No mission uses it right now.
- [ ] Depth uses a hardcoded atmospheric pressure (101325 Pa) -> up to ±20-30 cm error depending
      on weather. Calibrate on the surface at startup.
- [ ] Tune the PID gains (`pid/pid.py`).
- [ ] `blackboard_manager.py`: drop the stale TODO about `bluespark_navigation` (package removed);
      collect what else our sensors give us and put it on the blackboard (from the TODO in that file).
- [ ] `behaviours/control.py`: check what `MANUAL` mode really does on the robot (so far only tried in Gazebo).
- [ ] Later: state estimation node (calibrated depth, heading from `/mavros/vfr_hud`) that publishes
      on topics, so depth is visible in `ros2 topic echo` / rosbag.
- [ ] Later: control through the blackboard - behaviours write their axes, one place publishes all
      axes every tick (replaces per-axis service calls, fixes `MoveRC` overriding axes).
- [ ] Later: read `/side/detected_objects` too (now only `/front/...` reaches the blackboard).

---

## bluespark_control

- [ ] Three nodes in `vehicle_manager_node` get the same name from `name="vehicle_manager_node"`
      in `governer.launch.py`. Remove `name=` or merge them into one node.
- [ ] `StateListener` logs every `/mavros/state` message; log only on change.
- [ ] Confirm `RCMAP_*` params in QGC match the channel mapping in the README.
- [ ] Confirm the heave direction on the bench (> 1500 up or down?); old code comments disagreed.
- [ ] Decide if `vehicle_manager` (proxy to mavros) is needed, or `run_governer.sh` should call mavros directly.
- [ ] Cleanup: unused `import os` in `rc_override_node.py`, duplicate dead-man-switch TODO above
      `RCOverrideNode`, `stat_subscirber` typo, empty test files, `License decaration` typo in `setup.py`.
      The first point of the TODO block in `vehicle_manager_node.py` is outdated (the tree already checks).

---

## Payload (bluespark_peripherals, removed in 520869f)

Servo code for ball drops and the torpedo (PCA9685 over I2C) was removed because it was never tested.
It is in git at `520869f^` (`src/bluespark_peripherals/`, with a README). When it comes back:

- [ ] Test on hardware, tune servo angles (were hardcoded 90° / 0°), call `reset()` on startup.
- [ ] Decide which Pi has the PCA9685 and start `servo_node` from its launch file.
- [ ] Use a service that returns success instead of a one-shot topic (`/payload/action`), so the tree
      knows if the action happened (new `.srv` in `bluespark_interfaces`).
- [ ] Channels, I2C address and angles as ROS parameters; ROS logger instead of `print()`.

---

## bluespark_interfaces

- [ ] Decide with the vision team if `VisionTarget` stays as the interface for future down-camera detectors.
- [ ] `DetectedObject` has no id; `BlackboardManager` keys by `label`, so two objects of the same
      class in one frame overwrite each other.
- [ ] `package.xml`: add a description, maintainer is `root`.
- [ ] README: replace its `## TODO` section with a link to this file (it still mentions `bluespark_peripherals`).

---

## bluespark_bringup

- [ ] `down.launch.py` still starts `pipeline_node`, `path_marker_node`, `bin_detector_node`, which no
      longer exist (their entry points were removed in 7009075), so the down role fails. Make it camera-only.
- [ ] **Test who runs `rpicam-vid`.** The camera launches start it inside the container, but it was
      never installed in the image. Check on a camera Pi:
      `docker run --rm --entrypoint sh <image> -c 'which rpicam-vid || echo BRAK'`,
      `which rpicam-vid`, `ls /etc/systemd/system | grep -iE "camera|bluespark"`,
      `pgrep -a rpicam`, `sudo ss -ltnp | grep 5000`.
      Then either start the stream on the host (document the service, keep `run_camera_tcp.sh`,
      remove `ExecuteProcess` from the launches) or install `rpicam-apps` in the image.
- [ ] `governer.launch.py` docstring mentions the removed `benchTest` and `test03.sh`.
- [ ] `package.xml`: "brain" -> "governer". `side.launch.py` docstring says "front camera".
- [ ] Merge `front` / `side` / `down` into one `camera.launch.py` with parameters
      (the three files are identical except for names).
- [ ] Add a README (roles, launch arguments, examples).
- [ ] Later: bring back down-camera detectors (pipeline, path marker, bin). Old code is in git at
      `7a859a7^` (`src/bluespark_vision/bluespark_vision/detectors/` and `*_node.py`); they publish
      `bluespark_interfaces/VisionTarget`.

---

## bluespark_vision (for the vision team)

- [ ] `vision_node` loads `ml_models/best.pt`, which is gitignored (`*.pt`). It must be copied in by
      hand before `colcon build` on every camera Pi, otherwise `vision_node` crashes on start.
      Document where to get it.
- [ ] README still describes the removed down-camera detectors.
- [ ] `camera.py`: camera index is hardcoded to 0; proper check whether the Pi camera is available (FIXME).
- [ ] `image_utils.py`: magic numbers, needs comments (from the TODO in the file).

---

## Infra (Docker, compose, scripts, docs)

- [ ] `DEV_TEST.md` runs `benchTest ... skip_fcu:=true`, and `docker-compose.bench.yml` mentions
      `benchTest` in its header. `benchTest` was removed; update both (see `skip_fcu` above).
- [ ] `run_governer.sh` disarms through `/manager/set_arming`; if `vehicle_manager` is dead the
      disarm fails silently. Fall back to `/mavros/cmd/arming`.
- [ ] `run_governer.sh` starts with an unexplained `sleep 20`. Document why, or wait for
      `/dev/ttyACM0` / the network instead.
- [ ] Every container start runs `colcon build`; a commit that does not build means nothing comes
      up after a reboot.
- [ ] Containers run as root, so `build/`, `install/`, `log/` in the repo are owned by root
      (`sudo rm -rf build install log` to clean). For the dev compose consider `user: "${UID}:${GID}"`.
- [ ] Put `bluespark.service` (systemd unit that starts prod on boot) into the repo, e.g.
      `deploy/bluespark.service`, with install instructions. Today it only lives on the Pis.
- [ ] The governer image builds YOLO/torch it never uses. Add a build arg (e.g. `WITH_VISION=0/1`).
- [ ] Use the geographiclib script shipped with mavros
      (`/opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh`) instead of downloading it from master.
- [ ] `pip install lgpio` may fail to build on the Pi (Python 3.12, ARM); fallback: apt `python3-lgpio`.
- [ ] Remove legacy `docker-compose.yml` (old X11 compose, picked by a bare `docker compose up`)
      and point `run_docker_gui.sh` at `docker-compose.dev.yml`.
- [ ] Remove unused `steady_requirements.txt` (`opencv-python` conflicts with apt `python3-opencv`).
- [ ] `env.example`: says `cp .env.example .env` (file has no dot); `down` description is outdated.
- [ ] `docker-compose.prod.yml` header: the role/supervisor note is garbled
      ("role = governer, supervisor = governer"); add a warning that the governer auto-arms.
- [ ] Root `README.md`: still lists `libgl1-mesa-glx`, describes only vision/interfaces/control, no
      architecture and no run instructions.
