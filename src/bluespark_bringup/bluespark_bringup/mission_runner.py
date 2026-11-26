"""
mission_runner — runs ONE mission with full logging and a safety stop.

    ros2 run bluespark_bringup mission_runner <mission> [args for the mission]
    (./bs mission <mission> calls exactly this inside the container)

Needs the governer stack running (./bs start). Steps:
  1. checks that the Pixhawk is connected (else: exits, nothing armed),
  2. creates the run folder inside the session folder ($BS_SESSION_DIR):
         logs/<session>/<NN>_<HH-MM-SS>_<mission>/
             info.txt         what / when / result / git commit / safety stop
             console.log      everything the mission printed, with timestamps
             bag/             rosbag (mcap) of all topics except camera images
             bag_record.log   output of ros2 bag record
             ros_log/         ROS log files of the mission process
             git_diff.patch   only if the code had uncommitted changes
  3. records the bag, marks START / END in the session timeline.log,
  4. runs the mission (entry point of bluespark_autonomy) and waits for it to exit,
  5. ALWAYS (finished, failed, crashed, Ctrl+C, even an error of the runner
     itself or a lost terminal): disarm + all thrusters 1500.

Ctrl+C once: disarms + thrusters 1500 right away, asks the mission to stop
(SIGINT), and runs the safety stop again once it is gone.
Ctrl+C again: kills the mission immediately.
Closing the terminal (SIGHUP) is handled like Ctrl+C (not tested over a real SSH drop yet).

Result (info.txt, exit code of this command = exit code of the mission):
  OK (0), FAILED: setup (2), FAILED: a step of the mission failed (3),
  INTERRUPTED (Ctrl+C), KILLED (by a signal), FAILED (other exit codes = crash).
"""

import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from bluespark_bringup.timeline_recorder import TIMELINE_QOS, TIMELINE_TOPIC
from bluespark_bringup.vehicle import Vehicle

WORKSPACE = "/workspace"
LOGS_ROOT = os.environ.get("BS_LOGS_ROOT", os.path.join(WORKSPACE, "logs"))
IMAGE_TYPES = ["sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"]
ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
RESULTS = {  # exit codes of bluespark_autonomy.mission_runtime
    0: "OK",
    2: "FAILED: setup (services missing, see console.log)",
    3: "FAILED: a step of the mission failed (see timeline.log / console.log)",
}


def now_str():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def shown(path):
    """Path as the user sees it on the Pi (relative to the repo)."""
    return os.path.relpath(path, WORKSPACE) if path.startswith(WORKSPACE + "/") else path


def fmt_duration(sec):
    return f"{sec:.1f} s" if sec < 60 else f"{int(sec // 60)} min {sec % 60:.0f} s"


def mission_lib():
    """Folder with the mission executables (entry points of bluespark_autonomy)."""
    return os.path.join(get_package_prefix("bluespark_autonomy"), "lib", "bluespark_autonomy")


def available_missions():
    try:
        lib = mission_lib()
    except PackageNotFoundError:
        return []
    return sorted(f for f in os.listdir(lib) if not f.startswith((".", "_")))


def run_cmd(args):
    try:
        return subprocess.run(args, capture_output=True, text=True, timeout=10).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def git_info():
    git = ["git", "-C", WORKSPACE]
    commit = run_cmd(git + ["rev-parse", "--short", "HEAD"]) or "unknown"
    branch = run_cmd(git + ["rev-parse", "--abbrev-ref", "HEAD"]) or "?"
    status = run_cmd(git + ["status", "--porcelain"])
    diff = run_cmd(git + ["diff", "HEAD"]) if status else ""
    return commit, branch, status, diff


def session_dir():
    path = os.environ.get("BS_SESSION_DIR", "")
    if path and os.path.isdir(path):
        return path
    path = os.path.join(LOGS_ROOT, datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + "_no-session")
    os.makedirs(path, exist_ok=True)
    out(f"(no ./bs session found, logging to {shown(path)})")
    return path


def new_run_dir(session, mission):
    numbers = [int(d[:2]) for d in os.listdir(session) if re.match(r"^\d\d_", d)]
    number = max(numbers, default=0) + 1
    path = os.path.join(session, f"{number:02d}_{datetime.now():%H-%M-%S}_{mission}")
    os.makedirs(path)
    return number, path


def chown_like(path, reference):
    """The container runs as root; give the files to the owner of the session dir."""
    try:
        st = os.stat(reference)
        for root, dirs, files in os.walk(path):
            for name in [root] + [os.path.join(root, x) for x in dirs + files]:
                os.chown(name, st.st_uid, st.st_gid)
    except OSError:
        pass


def kill_group(proc, sig):
    try:
        os.killpg(proc.pid, sig)
    except (ProcessLookupError, PermissionError):
        pass


def wait(proc, timeout):
    try:
        proc.wait(timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        return False


def out(text="", end="\n"):
    """Print to the terminal, best effort: a lost terminal (SSH dropped, closed
    pipe) must never stop the runner before the safety stop."""
    try:
        sys.stdout.write(text + end)
        sys.stdout.flush()
    except (OSError, ValueError):
        pass


class MissionRunner:
    def __init__(self, mission, extra_args):
        self.mission = mission
        self.extra_args = extra_args
        self.proc = None
        self.interrupts = 0
        self.info = {}
        self.run_dir = None

    # --- signals: Ctrl+C / docker stop / SSH hang-up ---
    def on_signal(self, signum, _frame):
        self.interrupts += 1
        if self.proc is None or self.proc.poll() is not None:
            return
        if self.interrupts == 1:
            out("\n>>> Stopping the mission (Ctrl+C again = kill it now)...")
            kill_group(self.proc, signal.SIGINT)
        else:
            out("\n>>> Killing the mission now.")
            kill_group(self.proc, signal.SIGKILL)

    # --- info.txt ---
    def write_info(self):
        width = max(len(k) for k in self.info)
        with open(os.path.join(self.run_dir, "info.txt"), "w", encoding="utf-8") as f:
            for key, value in self.info.items():
                f.write(f"{key:<{width}}  {value}\n")

    def marker(self, text):
        try:
            self.timeline.publish(String(data=text))
        except Exception:  # noqa: BLE001 - logging must not break the run
            pass

    def pump_output(self, console):
        """Mission output -> terminal (as is) + console.log (timestamped, no colours)."""
        for line in iter(self.proc.stdout.readline, ""):
            out(line, end="")
            console.write(f"{datetime.now():%H:%M:%S.%f}"[:-3] + "  " + ANSI.sub("", line))
            console.flush()

    def run(self):
        missions = available_missions()
        if self.mission not in missions:
            out(f"Unknown mission '{self.mission}'. Available: {', '.join(missions) or 'none'}")
            return 2

        # args=[]: --ros-args on the command line are for the mission, not for us.
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("mission_runner")
        vehicle = Vehicle(node)
        self.timeline = node.create_publisher(String, TIMELINE_TOPIC, TIMELINE_QOS)
        try:
            # A fresh process needs a moment to discover the stack: wait up to 8 s.
            vehicle.spin_until(
                lambda: vehicle.state is not None and vehicle.state.connected, 8.0)
            state = vehicle.state
            if state is None or not state.connected:
                out("The stack is not running or the Pixhawk is not connected.")
                out("Start it with ./bs start, check with ./bs status.")
                return 4
            for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
                signal.signal(sig, self.on_signal)
            return self.run_logged(vehicle)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def run_logged(self, vehicle):
        session = session_dir()
        number, self.run_dir = new_run_dir(session, self.mission)
        commit, branch, status, diff = git_info()
        if diff:
            with open(os.path.join(self.run_dir, "git_diff.patch"), "w", encoding="utf-8") as f:
                f.write(f"# git status --porcelain\n{status}\n\n{diff}\n")
        executable = os.path.join(mission_lib(), self.mission)
        self.info = {
            "mission": self.mission,
            "run": f"{number:02d}",
            "result": "RUNNING (or the runner itself died)",
            "exit_code": "",
            "started": now_str(),
            "ended": "",
            "duration": "",
            "safe_stop": "",
            "command": " ".join([self.mission] + self.extra_args),
            "git": f"{commit} on {branch}, local changes: "
            + ("YES, see git_diff.patch" if status else "no"),
            "session": shown(session),
            "host": socket.gethostname(),
            "bag": "starting",
        }
        self.write_info()
        out(f"Run {number:02d}: {self.mission}   logs -> {shown(self.run_dir)}/")

        # Let the timeline recorder see our publisher before the first marker.
        vehicle.spin_until(lambda: self.timeline.get_subscription_count() > 0, 2.0)

        bag = None
        bag_log = open(os.path.join(self.run_dir, "bag_record.log"), "w")
        console = open(os.path.join(self.run_dir, "console.log"), "w", encoding="utf-8")
        code, result, duration = None, None, 0.0
        start = time.monotonic()
        try:
            # --- rosbag: everything except camera images ---
            bag = subprocess.Popen(
                ["ros2", "bag", "record", "--all-topics", "--exclude-topic-types", *IMAGE_TYPES,
                 "-s", "mcap", "-o", os.path.join(self.run_dir, "bag")],
                stdout=bag_log, stderr=subprocess.STDOUT, start_new_session=True,
            )
            time.sleep(2.0)  # bag subscribes to the topics
            if bag.poll() is not None:
                self.info["bag"] = "NOT RECORDED, see bag_record.log"
                out("WARNING: rosbag did not start, see bag_record.log. Running without it.")
            else:
                self.info["bag"] = "bag/ (all topics except camera images)"

            # --- the mission ---
            if self.interrupts:
                raise KeyboardInterrupt  # Ctrl+C before the mission even started
            env = dict(os.environ)
            env.update(
                PYTHONUNBUFFERED="1",
                RCUTILS_COLORIZED_OUTPUT="1" if sys.stdout.isatty() else "0",
                ROS_LOG_DIR=os.path.join(self.run_dir, "ros_log"),
            )
            self.marker(f"START mission {number:02d} {self.mission} (git {commit})")
            start = time.monotonic()
            self.proc = subprocess.Popen(
                [executable, *self.extra_args],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors="replace",
                env=env, start_new_session=True,
            )
            pump = threading.Thread(target=self.pump_output, args=(console,), daemon=True)
            pump.start()

            # Wait. On Ctrl+C: disarm RIGHT AWAY (do not wait for the mission to
            # close, it could hang with the thrusters on), then give the mission
            # 10 s to stop by itself before killing it.
            early_stop_done = False
            while self.proc.poll() is None:
                time.sleep(0.1)
                if self.interrupts and not early_stop_done:
                    early_stop_done = True
                    for _ok, text in vehicle.safe_stop():
                        out(f">>> {text}")
                    if not wait(self.proc, 10):
                        out(">>> The mission does not stop, killing it.")
                        kill_group(self.proc, signal.SIGKILL)
                        self.proc.wait()
            code = self.proc.returncode
            duration = time.monotonic() - start
            pump.join(timeout=2)
        except KeyboardInterrupt:
            result = "INTERRUPTED (Ctrl+C before the mission started)"
        except Exception as exc:  # noqa: BLE001 - recorded, then safety stop
            result = f"RUNNER ERROR: {exc!r}"
        finally:
            # Whatever happened above: the mission must be gone and the vehicle safe.
            if self.proc is not None and self.proc.poll() is None:
                kill_group(self.proc, signal.SIGKILL)
                self.proc.wait()
            stops = vehicle.safe_stop()
            console.close()

        if result is None:
            if self.interrupts:
                result = "INTERRUPTED (Ctrl+C)"
            elif code in RESULTS:
                result = RESULTS[code]
            elif code < 0:
                result = f"KILLED (signal {-code})"
            else:
                result = f"FAILED (exit code {code}, see console.log)"
        safe = "; ".join(text for _, text in stops)
        safe_ok = all(ok for ok, _ in stops)

        self.marker(f"END mission {number:02d} {self.mission}: {result} ({fmt_duration(duration)})")
        self.info.update(
            result=result,
            exit_code="" if code is None else str(code),
            ended=now_str(),
            duration=fmt_duration(duration),
            safe_stop=("OK: " if safe_ok else "PROBLEM: ") + safe,
        )
        self.write_info()

        if bag is not None:
            kill_group(bag, signal.SIGINT)
            if not wait(bag, 10):
                kill_group(bag, signal.SIGKILL)
        bag_log.close()
        chown_like(self.run_dir, session)

        line = "=" * 60
        out(f"\n{line}\n {self.mission}: {result}   ({fmt_duration(duration)})")
        out(f" safety stop: {'OK' if safe_ok else 'PROBLEM'}: {safe}")
        out(f" logs: {shown(self.run_dir)}/\n{line}")
        if not safe_ok:
            out("!!! Check the vehicle: ./bs status, ./bs disarm")
        if self.interrupts:
            return 130
        return code if code is not None else 1


def main():
    args = sys.argv[1:]  # passed to the mission as they are (incl. --ros-args ...)
    if not args or args[0] in ("-h", "--help"):
        out(__doc__)
        out("Available missions: " + (", ".join(available_missions()) or "none"))
        sys.exit(0 if args else 2)
    sys.exit(MissionRunner(args[0], args[1:]).run())


if __name__ == "__main__":
    main()
