sleep 5

source /opt/ros/humble/setup.bash

WORKSPACE=$(dirname $(realpath $0))
source $WORKSPACE/install/setup.bash

PIDS=()


start_all() {
    PIDS=()

    echo "-- Startuje wezly --"

    ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200 &
    PIDS+=($!)
    sleep 3

    ros2 run bluespark_control rc_override_node &
    PIDS+=($!)
    sleep 2

    ros2 run bluespark_control vehicle_manager_node &
    PIDS+=($!)
    sleep 2

    # ros2 run bluespark_vision vision_node &
    # PIDS+=($!)
    # sleep 2

    # ros2 run bluespark_navigation movement_node &
    # PIDS+=($!)
    # sleep 2 
    
    ros2 run bluespark_navigation depth_hold_node &
    PIDS+=($!)
    sleep 2

    ros2 run bluespark_navigation depth_estimator_node &
    PIDS+=($!)

    echo "-- Czekam 5 sekund na polaczenie z MAVROS... --"
    sleep 5
    
    echo "-- Ustawiam tryb ALT_HOLD i uzbrajam drona --"
    ros2 service call /manager/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'ALT_HOLD'}"
    sleep 1
    ros2 service call /manager/set_arming std_srvs/srv/SetBool "{data: true}"
    

    echo "-- Wszystkie wezly wstaly --"
}

stop_all() {
    echo "-- Zatrzymuje dzialanie wszystkich wezlow --"

    pkill -f "rc_override_node" 2>/dev/null
    pkill -f "vehicle_manager_node" 2>/dev/null
    pkill -f "vision_node" 2>/dev/null
    pkill -f "movement_node" 2>/dev/null
    pkill -f "depth_hold_node" 2>/dev/null

    sleep 2

    PIDS=()
}

check_all() {
    for pid in "${PIDS[@]}"; do

        if ! kill -0 "$pid" 2>/dev/null; then
            echo "-- Wezel o PID $pid nie zyje --"
            return 1
        fi
    done
    return 0
}

cleanup() {
    stop_all
    exit 0
}
trap cleanup SIGINT SIGTERM

while true; do
    start_all

    while check_all; do
        sleep 1
    done

    echo "-- Wezel nie dziala. Restart --"

    stop_all
    sleep 3
done
