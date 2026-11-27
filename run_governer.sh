#!/bin/bash
#
# run_governer.sh — supervisor for the governer (main) Raspberry Pi.
#
# Replaces the old manual "start every node" scripts with a single
# `ros2 launch governer.launch.py`, while KEEPING the safety/lifecycle logic that
# launch does not provide:
#   - distinguish a clean mission finish (rc=0) from a crash (rc!=0)
#   - on clean finish (mission rc 0): disarm, stop the stack and stay idle
#     (do not exit: docker would restart the container and fly the mission again)
#   - on crash: disarm for safety, then restart
#   - restart-storm guard: give up if the stack keeps dying too fast
#   - always disarm + set MANUAL before restarting or on Ctrl+C
#
# The launch file owns "what to start, in what order". This script owns
# "what to do when it stops". That split is deliberate: safety logic stays
# explicit in bash instead of hidden in launch event handlers.
#
sleep 20

WORKSPACE=$(dirname "$(realpath "$0")")

echo "-- Loading base ROS 2 environment --"
source /opt/ros/jazzy/setup.bash

echo "-- Building the workspace --"
cd "$WORKSPACE"
colcon build --symlink-install

echo "-- Loading built BlueSpark environment --"
source "$WORKSPACE/install/setup.bash"

# Session log, like ./bs sessions: timeline_recorder writes every node's log
# line and every /mavros/state and thruster change to $SESSION/timeline.log.
# One folder per container start; restarts after a crash append to it. Plain
# text, a few MB per hour at most.
SESSION="$WORKSPACE/logs/$(date +%Y-%m-%d_%H-%M-%S)_prod"
mkdir -p "$SESSION"
{
    echo "started   $(date '+%Y-%m-%d %H:%M:%S')"
    echo "host      $(hostname)"
    echo "mode      prod  (run_governer.sh)"
    echo "git       $(git -C "$WORKSPACE" rev-parse --short HEAD 2>/dev/null) on $(git -C "$WORKSPACE" rev-parse --abbrev-ref HEAD 2>/dev/null)"
} > "$SESSION/session_info.txt"
echo "-- Session log: $SESSION/timeline.log --"

# The single launch process we supervise (instead of many node PIDs).
LAUNCH_PID=""

# Restart-storm guard: how many times the stack may die faster than MIN_UPTIME
# before we give up.
MAX_FAST_CRASHES=5
MIN_UPTIME=15          # seconds — shorter than this counts as a "fast crash"
fast_crash_count=0

start_stack() {
    echo "-- Launching governer stack --"
    # One launch brings up mavros + control + autonomy at once; the mission
    # waits for the FCU and its services itself. The launch ends when the
    # mission ends: rc 0 = mission OK, rc != 0 = failed / crashed.
    ros2 launch bluespark_bringup governer.launch.py log_dir:="$SESSION" &
    LAUNCH_PID=$!
    echo "-- Launch started (PID $LAUNCH_PID) --"
}

stop_stack_gracefully() {
    echo "-- Asking the launch to shut down (SIGINT) --"
    # SIGINT to the launch triggers an orderly shutdown of all its nodes.
    if [ -n "$LAUNCH_PID" ]; then
        kill -SIGINT "$LAUNCH_PID" 2>/dev/null
        # Give launch time to bring its nodes down cleanly.
        for _ in $(seq 1 10); do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 0.5
        done
        # Force-kill if it is still alive.
        kill -9 "$LAUNCH_PID" 2>/dev/null
    fi
    # Belt-and-suspenders: make sure no stragglers survive.
    pkill -f "mavros_node"          2>/dev/null
    pkill -f "rc_override_node"     2>/dev/null
    pkill -f "timeline_recorder"    2>/dev/null
    pkill -f "autonomy_node"        2>/dev/null
}

# Uses the `vehicle` CLI of bluespark_bringup (same as ./bs disarm): it talks to
# mavros directly and confirms the result on /mavros/state, so a failed disarm
# is printed as [FAIL] instead of going unnoticed. If it fails (including our
# own code being broken), a plain `ros2 service call` to mavros is tried too.
disarm() {
    local vehicle
    vehicle="$(ros2 pkg prefix bluespark_bringup)/lib/bluespark_bringup/vehicle"
    # Normally each takes ~1-2 s. The timeouts only matter when something is
    # broken; they are long enough for `vehicle` to print WHY it failed, and keep
    # cleanup within stop_grace_period in docker-compose.prod.yml (40 s):
    # disarm 15 s + fallback 10 s (a fresh `ros2 service call` needs ~6 s)
    # + mode 8 s + stopping the launch 5 s.
    echo "-- Disarming the vehicle + thrusters to 1500 --"
    if ! timeout 15 "$vehicle" disarm; then
        echo "-- WARNING: disarm NOT confirmed, check the vehicle! Trying mavros directly --"
        timeout 10 ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
            "{value: false}" | grep -E "success|result"
    fi

    echo "-- Setting MANUAL mode --"
    timeout 8 "$vehicle" mode MANUAL || echo "-- WARNING: MANUAL mode NOT confirmed --"
}

cleanup() {
    echo "-- Ctrl+C received in terminal --"
    disarm
    stop_stack_gracefully
    exit 0
}

trap cleanup SIGINT SIGTERM

while true; do
    start_stack
    run_start=$(date +%s)

    # Supervise: block until the launch process exits, then read its exit code.
    wait "$LAUNCH_PID"
    rc=$?

    run_end=$(date +%s)
    uptime=$((run_end - run_start))

    # Clean finish (rc=0): mission done -> tidy up and stay idle. Do NOT exit:
    # docker-compose.prod.yml has restart: unless-stopped, so exiting would
    # restart the container and run the mission again. Start a new run with
    # `docker restart bluespark_governer`.
    if [ "$rc" -eq 0 ]; then
        echo "-- Launch exited cleanly (rc=0). Mission complete. --"
        disarm
        stop_stack_gracefully
        echo "-- Idle. Run the mission again with: docker restart bluespark_governer --"
        LAUNCH_PID=""
        sleep infinity &
        wait $!   # interruptible: SIGINT/SIGTERM still run cleanup()
        exit 0
    fi

    # rc != 0: something crashed. Disarm, clean up, maybe restart.
    echo "-- Launch died after ${uptime}s (rc=$rc). Cleaning up... --"
    disarm
    stop_stack_gracefully

    # Restart-storm guard.
    if [ "$uptime" -lt "$MIN_UPTIME" ]; then
        fast_crash_count=$((fast_crash_count + 1))
        echo "-- Fast crash ($fast_crash_count/$MAX_FAST_CRASHES) --"
        if [ "$fast_crash_count" -ge "$MAX_FAST_CRASHES" ]; then
            echo "-- Too many fast crashes in a row. Something is permanently broken. Exiting. --"
            exit 1
        fi
    else
        # Ran long enough to be a real runtime crash — reset the counter.
        fast_crash_count=0
    fi

    echo "-- Restarting in 3s --"
    sleep 3
done
