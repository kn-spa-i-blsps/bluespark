#!/bin/bash
#
# run_governer.sh — supervisor for the brain (main) Raspberry Pi.
#
# Replaces the manual "start eight nodes" part of test03.sh with a single
# `ros2 launch governer.launch.py`, while KEEPING the safety/lifecycle logic that
# launch does not provide:
#   - distinguish a clean mission finish (rc=0) from a crash (rc!=0)
#   - on clean finish: stop and exit (do not restart forever)
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
source /opt/ros/humble/setup.bash

echo "-- Building the workspace --"
cd "$WORKSPACE"
colcon build --symlink-install

echo "-- Loading built BlueSpark environment --"
source "$WORKSPACE/install/setup.bash"

# The single launch process we supervise (instead of many node PIDs).
LAUNCH_PID=""

# Restart-storm guard: how many times the stack may die faster than MIN_UPTIME
# before we give up.
MAX_FAST_CRASHES=5
MIN_UPTIME=15          # seconds — shorter than this counts as a "fast crash"
fast_crash_count=0

start_stack() {
    echo "-- Launching governer stack --"
    # One launch brings up mavros + control + autonomy in order.
    ros2 launch bluespark_bringup governer.launch.py &
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
    pkill -f "vehicle_manager_node" 2>/dev/null
    pkill -f "autonomy_node"        2>/dev/null
}

disarm() {
    echo "-- Disarming the vehicle --"
    timeout 3 ros2 service call /manager/set_arming std_srvs/srv/SetBool \
        "{data: false}" 2>/dev/null

    echo "-- Setting MANUAL mode --"
    timeout 3 ros2 service call /manager/set_mode mavros_msgs/srv/SetMode \
        "{custom_mode: 'MANUAL'}" 2>/dev/null
    sleep 1
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

    # Clean finish (rc=0): mission done -> tidy up and exit for good.
    if [ "$rc" -eq 0 ]; then
        echo "-- Launch exited cleanly (rc=0). Mission complete. Exiting. --"
        disarm
        stop_stack_gracefully
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