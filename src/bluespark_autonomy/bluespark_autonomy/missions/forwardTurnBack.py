"""
forwardTurnBack — swim forward, turn 180°, swim back.

ALT_HOLD (Pixhawk) -> arm -> surge forward_sec -> stop -> yaw turn_sec (~180°)
-> stop -> surge forward_sec (back) -> stop -> disarm.

Parameters (defaults):  ./bs mission forwardTurnBack --ros-args -p turn_sec:=3.5
    surge_pwm    1600   forward speed
    forward_sec  5.0    time forward (each way)
    yaw_pwm      1600   turning speed
    turn_sec     4.0    time turning: tune until it is 180°
    pause_sec    2.0    stop between the moves
"""

from py_trees.composites import Sequence

from bluespark_autonomy.behaviours.hardcoded_actions import MoveRC
from bluespark_autonomy.mission_runtime import run_mission
from bluespark_autonomy.missions.common import finish, params, pause, start_in_althold

DEFAULTS = {
    "surge_pwm": 1600,
    "forward_sec": 5.0,
    "yaw_pwm": 1600,
    "turn_sec": 4.0,
    "pause_sec": 2.0,
}


def create_tree(node):
    p = params(node, DEFAULTS)
    mission = Sequence(name="Forward, Turn 180, Back", memory=True)
    start_in_althold(mission)
    mission.add_child(MoveRC("Swim Forward", duration_sec=p["forward_sec"], surge=p["surge_pwm"]))
    mission.add_child(pause("Stop 1", p["pause_sec"]))
    mission.add_child(MoveRC("Turn 180", duration_sec=p["turn_sec"], yaw=p["yaw_pwm"]))
    mission.add_child(pause("Stop 2", p["pause_sec"]))
    mission.add_child(MoveRC("Swim Back", duration_sec=p["forward_sec"], surge=p["surge_pwm"]))
    finish(mission)
    return mission


def main(args=None):
    run_mission("forward_turn_back", create_tree, args=args)


if __name__ == "__main__":
    main()
