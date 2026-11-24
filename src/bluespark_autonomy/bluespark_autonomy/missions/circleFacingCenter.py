"""
circleFacingCenter — swim one circle sideways, front always facing the center.

ALT_HOLD (Pixhawk) -> arm -> sway + yaw together for circle_sec -> stop -> disarm.
Sideways movement plus turning the opposite way keeps the front pointed at the
center of the circle.

Parameters (defaults):  ./bs mission circleFacingCenter --ros-args -p circle_sec:=25.0
    sway_pwm     1600   sideways speed
    yaw_pwm      1400   turning speed, the OTHER side of 1500 than sway_pwm.
                        Front drifts away from the center? Use 1600 (and keep sway).
    circle_sec   20.0   time for one full circle: tune it
"""

from py_trees.composites import Sequence

from bluespark_autonomy.behaviours.hardcoded_actions import MoveRC
from bluespark_autonomy.mission_runtime import run_mission
from bluespark_autonomy.missions.common import finish, params, start_in_althold

DEFAULTS = {
    "sway_pwm": 1600,
    "yaw_pwm": 1400,
    "circle_sec": 20.0,
}


def create_tree(node):
    p = params(node, DEFAULTS)
    mission = Sequence(name="Circle, Facing the Center", memory=True)
    start_in_althold(mission)
    mission.add_child(MoveRC("Circle Sideways", duration_sec=p["circle_sec"],
                             sway=p["sway_pwm"], yaw=p["yaw_pwm"]))
    finish(mission)
    return mission


def main(args=None):
    run_mission("circle_facing_center", create_tree, args=args)


if __name__ == "__main__":
    main()
