#!/usr/bin/env python3

from time import sleep
from signal import signal, SIGINT, SIGTERM
import subprocess


procs = {}

def check_all():
    for name, proc in procs.items():
        if proc.poll() is not None:
            print(f"-- Wezel {name} padl --")
            print(f"PID: {proc.pid}")
            return False
    return True

def stop_all_gracefully():
    # How long the program waits for the nodes to gracefully exit
    GRACE_PERIOD = 3
    print("-- Prosze wszystkie wezly o wylaczenie --")

    nodes = [
        "rc_override_node",
        "vehicle_manager_node",
        "depth_hold_node",
        "depth_estimator_node"
    ]
    for name in nodes:
        proc = procs[name]
        proc.send_signal(SIGINT)

    sleep(GRACE_PERIOD)

    for name in nodes:
        proc = procs[name]
        if proc.poll() is None:
            print(f"-- Wezel {name} nie wylaczyl sie --")
            proc.signal(SIGTERM)
    
    

def cleanup(signum, frame):
    print("-- Otrzymano Ctrl+C w terminalu --")
    stop_all_gracefully()
    stop_mavros()
    exit(0)

def start_all():
    global pids
    pids = {}
    mavros_proc = subprocess.Popen([
        "ros2", "launch", "mavros", "apm.launch",
        "fcu_url:=serial:///dev/ttyACM0:115200"
    ])
    procs["mavros"] = mavros_proc
    sleep(3)

    rc_override_node_proc = subprocess.Popen([
        "ros2", "run", "bluespark_control",
        "rc_override_node"
    ])
    procs["rc_override_node"] = rc_override_node_proc

    vehicle_manager_node_proc = subprocess.Popen([
        "ros2", "run", "bluespark_control",
        "vehicle_manager_node"
    ])
    procs["vehicle_manager_node"] = vehicle_manager_node_proc

    depth_hold_node_proc = subprocess.Popen([
        "ros2", "run", "bluespark_navigation",
        "depth_hold_node"
    ])
    procs["depth_hold_node"] = depth_hold_node_proc

    depth_estimator_node_proc = subprocess.Popen([
        "ros2", "run", "bluespark_navigation",
        "depth_estimator_node"
    ])
    procs["depth_estimator_node"] = depth_estimator_node_proc

    sleep(5)
    
    subprocess.run([
        "ros2", "service", "call",
        "/manager/set_mode",
        "mavros_msgs/srv/SetMode",
        "{custom_mode: 'ALT_HOLD'}"
    ])
    
    sleep(1)
    
    subprocess.run([
        "ros2", "service", "call",
        "/manager/set_arming",
        "std_srvs/srv/SetBool",
        "{data: true}"
    ])

    print("-- Wszystkie wezly wstaly --")

def disarm():
    print("-- Rozbrajam drona --")
    try: 
        subprocess.run([
            "ros2", "service", "call",
            "/manager/set_arming",
            "std_srvs/srv/SetBool",
            "{data: false}"
        ], timeout=3)
    except subprocess.TimeoutExpired:
        print("-- Nie udalo sie rozbrajac drona (timeout) --")

    print("-- Ustawiam tryb manual --")
    try:
        subprocess.run([
            "ros2", "service", "call",
            "/manager/set_mode",
            "mavros_msgs/srv/SetMode",
            "{custom_mode: 'MANUAL'}"
        ], 
        timeout=3,
        stderr=subprocess.DEVNULL)
    except subprocess.TimeoutExpired:
        print("-- Nie udalo sie ustawic trybu manual (timeout) --")

def stop_mavros():
    print("-- Zatrzymuje MAVROS --")
    # TODO: use the handles stored in the dict?
    subprocess.run(["pkill", "-f", "mavros_node"], stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "apm.launch"], stderr=subprocess.DEVNULL)
    
    

def main():
    sleep(5)
    signal(SIGINT, cleanup)


    while True:
        start_all()

        while check_all():
            sleep(1)

        disarm()
        stop_all_gracefully()
        stop_mavros()
        sleep(3)

if __name__ == "__main__":
    main()
