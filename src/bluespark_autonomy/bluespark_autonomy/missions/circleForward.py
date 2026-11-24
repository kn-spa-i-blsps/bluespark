"""
circleForward — swim one circle front first (facing the direction of travel).

ALT_HOLD (Pixhawk) -> arm -> surge + yaw together for circle_sec -> stop -> disarm.

Parameters (defaults):  ./bs mission circleForward --ros-args -p circle_sec:=25.0
    surge_pwm    1600   forward speed
    yaw_pwm      1600   turning speed (bigger difference from 1500 = smaller circle)
    circle_sec   20.0   time for one full circle: tune it
"""

from py_trees.composites import Sequence

from bluespark_autonomy.behaviours.hardcoded_actions import MoveRC
from bluespark_autonomy.mission_runtime import run_mission
from bluespark_autonomy.missions.common import finish, params, start_in_althold

DEFAULTS = {
    "surge_pwm": 1600,
    "yaw_pwm": 1600,
    "circle_sec": 20.0,
}


def create_tree(node):
    p = params(node, DEFAULTS)
    mission = Sequence(name="Circle, Front First", memory=True)
    start_in_althold(mission)
    mission.add_child(MoveRC("Circle Forward", duration_sec=p["circle_sec"],
                             surge=p["surge_pwm"], yaw=p["yaw_pwm"]))
    finish(mission)
    return mission


def main(args=None):
    run_mission("circle_forward", create_tree, args=args)


if __name__ == "__main__":
    main()
