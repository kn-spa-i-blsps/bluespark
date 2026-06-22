#!/bin/bash
#
# test03.sh — launcher dla autonomy stack BlueSpark
#
# Zmiany vs test02.sh:
#   - check_all odroznia czysty exit (rc=0, misja skonczona) od crasha
#   - przy czystym koncu skrypt KONCZY sie, nie restartuje w nieskonczonosc
#   - ochrona przed restart-storm: jesli wezel pada zbyt szybko kilka razy
#     z rzedu, skrypt rezygnuje zamiast palic CPU w petli
#   - bezpieczne disarm tylko gdy faktycznie restartujemy po awarii
#
sleep 5

WORKSPACE=$(dirname "$(realpath "$0")")
source "$WORKSPACE/install/setup.bash"

PIDS=()

# Ile razy z rzedu wezel moze paść szybciej niz MIN_UPTIME zanim sie poddamy
MAX_FAST_CRASHES=5
MIN_UPTIME=10          # sekundy — krocej niz to liczymy jako "szybki crash"
fast_crash_count=0

start_all() {
    PIDS=()

    echo "-- Startuje wezly --"

    # ros2 launch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200 &
    # PIDS+=($!)
    # sleep 3

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

    # ros2 run bluespark_navigation depth_hold_node &
    # PIDS+=($!)
    # sleep 2

    # ros2 run bluespark_navigation depth_estimator_node &
    # PIDS+=($!)
    # sleep 2

    echo "-- Czekam 5 sekund na polaczenie z MAVROS... --"
    sleep 5

    echo "-- Startuje wezel py_trees (autonomy_node) --"
    ros2 run bluespark_autonomy autonomy_node &
    PIDS+=($!)

    echo "-- Wszystkie wezly wstaly --"
}

stop_all_gracefully() {
    echo "-- Wysylam SIGINT do wezlow (prosze je o wylaczenie) --"

    pkill -SIGINT -f "rc_override_node" 2>/dev/null
    pkill -SIGINT -f "vehicle_manager_node" 2>/dev/null
    pkill -SIGINT -f "depth_hold_node" 2>/dev/null
    pkill -SIGINT -f "depth_estimator_node" 2>/dev/null
    pkill -SIGINT -f "autonomy_node" 2>/dev/null

    sleep 2

    # Ostateczne zabicie procesow, jesli nie zareagowaly na SIGINT
    pkill -f "rc_override_node" 2>/dev/null
    pkill -f "vehicle_manager_node" 2>/dev/null
    pkill -f "depth_hold_node" 2>/dev/null
    pkill -f "depth_estimator_node" 2>/dev/null
    pkill -f "autonomy_node" 2>/dev/null
}

cleanup() {
    echo "-- Otrzymano Ctrl+C w terminalu --"
    disarm
    stop_all_gracefully
    exit 0
}

disarm() {
    echo "-- Rozbrajam drona --"
    timeout 3 ros2 service call /manager/set_arming std_srvs/srv/SetBool \
        "{data: false}" 2>/dev/null

    echo "-- Ustawiam tryb MANUAL --"
    timeout 3 ros2 service call /manager/set_mode mavros_msgs/srv/SetMode \
        "{custom_mode: 'MANUAL'}" 2>/dev/null
    sleep 1
}

# Zwraca:
#   0 — wszystkie wezly zyja
#   1 — ktorys wezel padl z bledem (crash) -> restart
#   2 — ktorys wezel zakonczyl sie czysto (rc=0) -> koniec misji, nie restartuj
check_all() {
    for pid in "${PIDS[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid" 2>/dev/null
            local rc=$?
            if [ "$rc" -eq 0 ]; then
                echo "-- Wezel PID $pid zakonczyl sie poprawnie (rc=0) --"
                return 2
            else
                echo "-- Wezel PID $pid padl (rc=$rc) --"
                return 1
            fi
        fi
    done
    return 0
}

trap cleanup SIGINT SIGTERM

while true; do
    start_all
    run_start=$(date +%s)

    # Petla nadzoru: kreci sie dopoki check_all zwraca 0
    while check_all; do
        sleep 1
    done
    rc=$?

    run_end=$(date +%s)
    uptime=$((run_end - run_start))

    # Misja zakonczona poprawnie — sprzatamy i wychodzimy
    if [ "$rc" -eq 2 ]; then
        echo "-- Misja zakonczona poprawnie. Sprzatam i koncze. --"
        disarm
        stop_all_gracefully
        exit 0
    fi

    # Tu rc == 1: crash. Sprzatamy i ewentualnie restartujemy.
    echo "-- Wezel padl po ${uptime}s. Sprzatam... --"
    disarm
    stop_all_gracefully

    # Ochrona przed restart-storm
    if [ "$uptime" -lt "$MIN_UPTIME" ]; then
        fast_crash_count=$((fast_crash_count + 1))
        echo "-- Szybki crash ($fast_crash_count/$MAX_FAST_CRASHES) --"
        if [ "$fast_crash_count" -ge "$MAX_FAST_CRASHES" ]; then
            echo "-- Za duzo szybkich crashy z rzedu. Cos jest trwale zepsute. Koncze. --"
            exit 1
        fi
    else
        # Wezel pochodzil dluzej, wiec to byl realny runtime crash — resetuj licznik
        fast_crash_count=0
    fi

    echo "-- Restart za 3s --"
    sleep 3
done
