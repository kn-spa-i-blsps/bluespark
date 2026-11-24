"""
vehicle — manual control of the running governer stack, one short command each.

    ros2 run bluespark_bringup vehicle status
    ros2 run bluespark_bringup vehicle arm
    ros2 run bluespark_bringup vehicle disarm          # disarm + thrusters to 1500
    ros2 run bluespark_bringup vehicle mode MANUAL
    ros2 run bluespark_bringup vehicle pwm surge 1600
    ros2 run bluespark_bringup vehicle neutral         # all thrusters to 1500
    ros2 run bluespark_bringup vehicle test            # whole chain, step by step

./bs arm / disarm / mode / pwm / neutral / status / althold / test call exactly this.
Exit code 0 = done and confirmed, 1 = failed (the reason is printed).
"""

import argparse
import os
import sys

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String

from bluespark_bringup.timeline_recorder import TIMELINE_QOS, TIMELINE_TOPIC
from bluespark_bringup.vehicle import AXES, NEUTRAL, PWM_MAX, PWM_MIN, Vehicle, check_pwm

# ./bs test: order of the thruster checks, PWM step and time per axis.
TEST_AXES = ("surge", "sway", "heave", "yaw", "roll", "pitch")
TEST_STEP = 100  # 1500 + 100 = 1600, gentle
TEST_SECONDS = 2.0


def ok_line(ok, text):
    print(f"[{' OK ' if ok else 'FAIL'}] {text}", flush=True)
    return ok


def cmd_status(v, _args):
    state = v.wait_state(5.0)
    v.spin_until(lambda: v.rc is not None and v.battery is not None, 1.5)
    present, missing = v.stack_nodes()

    if state is None:
        print("Pixhawk    no /mavros/state (is the stack running? ./bs start)")
    else:
        print(f"Pixhawk    {'connected' if state.connected else 'NOT CONNECTED'}")
        print(f"Armed      {'YES' if state.armed else 'no'}")
        print(f"Mode       {state.mode or '?'}")
    if v.rc is not None:
        pwm = "  ".join(
            f"{axis} {'released' if value == 0 else value}"
            for axis, value in zip(AXES, v.rc.channels)
        )
        print(f"Thrusters  {pwm}")
    else:
        print("Thrusters  no /mavros/rc/override (rc_override_node not running?)")
    if v.battery is not None:
        print(f"Battery    {v.battery.voltage:.2f} V")
    else:
        print("Battery    n/a")
    print(f"Nodes      {', '.join(present) or 'none'}")
    if missing:
        print(f"MISSING    {', '.join(missing)}")
    return state is not None and state.connected and not missing


def cmd_arm(v, _args):
    v.wait_state(3.0)
    return ok_line(*v.set_armed(True))


def cmd_disarm(v, _args):
    v.wait_state(3.0)
    results = [ok_line(ok, text) for ok, text in v.safe_stop()]
    return all(results)


def cmd_mode(v, args):
    v.wait_state(3.0)
    return ok_line(*v.set_mode(args.mode))


def cmd_pwm(v, args):
    try:
        check_pwm(args.axis, args.value)
    except ValueError as exc:
        return ok_line(False, str(exc))
    return ok_line(*v.set_pwm({args.axis: args.value}))


def cmd_neutral(v, _args):
    return ok_line(*v.neutral())


def cmd_test(v, args):
    """Whole chain, step by step. Thrusters WILL spin."""
    pwm = NEUTRAL + args.step
    try:
        check_pwm("surge", pwm)
    except ValueError as exc:
        return ok_line(False, f"--step {args.step}: {exc}")
    timeline = v.node.create_publisher(String, TIMELINE_TOPIC, TIMELINE_QOS)
    v.spin_until(lambda: timeline.get_subscription_count() > 0, 1.0)
    steps = []  # (name, ok, detail)

    def step(name, ok, detail=""):
        steps.append((name, ok, detail))
        number = f"{len(steps):>2}."
        print(f"{number} {name:<28} {'OK ' if ok else 'FAIL'}  {detail}", flush=True)
        return ok

    print(f"Bench test: thrusters WILL spin ({pwm} for {args.seconds:g} s per axis). "
          "Ctrl+C = stop + disarm.\n", flush=True)
    timeline.publish(String(data="START bench test (./bs test)"))
    try:
        present, missing = v.stack_nodes()
        if not step("Stack nodes running", not missing,
                    f"missing: {', '.join(missing)}" if missing else ", ".join(present)):
            return finish(v, timeline, steps)
        state = v.wait_state(5.0)
        if not step("Pixhawk connected", bool(state and state.connected)):
            return finish(v, timeline, steps)

        step("Mode MANUAL", *v.set_mode("MANUAL"))
        step("Arm", *v.set_armed(True))

        for axis in TEST_AXES:
            index = AXES.index(axis)
            ok, text = v.set_pwm({axis: pwm})
            if ok:  # really sent to the Pixhawk?
                ok = v.spin_until(
                    lambda: v.rc is not None and v.rc.channels[index] == pwm, 2.0)
                text = f"{pwm} seen on /mavros/rc/override" if ok else \
                    f"{pwm} NOT seen on /mavros/rc/override"
            v.spin_until(lambda: False, args.seconds)  # let it spin
            back, back_text = v.set_pwm({axis: NEUTRAL})
            step(f"{axis} {pwm} for {args.seconds:g} s", ok and back,
                 text if back else back_text)

        step("Mode ALT_HOLD", *v.set_mode("ALT_HOLD"))
        step("Mode MANUAL", *v.set_mode("MANUAL"))
    except KeyboardInterrupt:
        print("\n>>> Interrupted.", flush=True)
        steps.append(("Interrupted (Ctrl+C)", False, ""))
    return finish(v, timeline, steps)


def finish(v, timeline, steps):
    """Always: disarm + neutral, then the summary."""
    ok_stop = all(ok for ok, _ in v.safe_stop())
    passed = sum(1 for _, ok, _ in steps if ok)
    failed = [name for name, ok, _ in steps if not ok]
    print(f"{'':3} {'Disarm + thrusters 1500':<28} {'OK ' if ok_stop else 'FAIL'}", flush=True)
    total_ok = not failed and ok_stop
    summary = f"{passed}/{len(steps)} OK" + (f", FAILED: {', '.join(failed)}" if failed else "")
    print(f"\nResult: {'ALL OK' if total_ok else 'PROBLEMS'}  ({summary})", flush=True)
    timeline.publish(String(data=f"END bench test: {summary}"))
    v.spin_until(lambda: False, 0.3)  # let the marker go out
    return total_ok


def main(argv=None):
    argv = remove_ros_args(sys.argv if argv is None else argv)[1:]
    parser = argparse.ArgumentParser(prog="vehicle", description="Manual control of the governer stack.")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status", help="connection, armed, mode, thrusters, battery, nodes")
    sub.add_parser("arm", help="arm (answer from the Pixhawk)")
    sub.add_parser("disarm", help="disarm + all thrusters to 1500")
    p = sub.add_parser("mode", help="set flight mode, e.g. MANUAL, STABILIZE, ALT_HOLD")
    p.add_argument("mode")
    p = sub.add_parser("pwm", help=f"set one axis: {', '.join(AXES)}; {PWM_MIN}-{PWM_MAX}, 1500 = stop")
    p.add_argument("axis", choices=AXES)
    p.add_argument("value", type=int)
    sub.add_parser("neutral", help="all thrusters to 1500")
    p = sub.add_parser("test", help="whole chain step by step (thrusters WILL spin)")
    p.add_argument("--step", type=int, default=TEST_STEP, help="PWM above 1500 (default 100)")
    p.add_argument("--seconds", type=float, default=TEST_SECONDS, help="per axis (default 2)")
    args = parser.parse_args(argv)

    commands = {
        "status": cmd_status,
        "arm": cmd_arm,
        "disarm": cmd_disarm,
        "mode": cmd_mode,
        "pwm": cmd_pwm,
        "neutral": cmd_neutral,
        "test": cmd_test,
    }
    # No rclpy signal handler: Ctrl+C must raise KeyboardInterrupt while the
    # node still works, so the commands can disarm on the way out.
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(f"vehicle_cli_{os.getpid()}")
    try:
        ok = commands[args.command](Vehicle(node), args)
    except KeyboardInterrupt:
        ok = False
        if args.command in ("arm", "pwm"):  # interrupted while acting: make it safe
            for _ok, text in Vehicle(node).safe_stop():
                print(text, flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
