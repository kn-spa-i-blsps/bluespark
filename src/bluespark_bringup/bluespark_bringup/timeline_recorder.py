"""
timeline_recorder — writes ONE human-readable, chronological log of a session.

Started by governer.launch.py when log_dir:=<dir> is given (./bs start does
that), before anything else, so it also catches mavros starting up. Writes
<log_dir>/timeline.log, one line per event, from all processes together:

    01:31:10.330  =====  START mission 01 forwardTurnBack (git 4e95764)  =====
    01:31:15.754  STATE  mode MANUAL -> ALT_HOLD
    01:31:16.753  STATE  ARMED
    01:31:17.527  INFO   [forward_turn_back_node] [tree] Swim Forward: RUNNING -> SUCCESS
    01:31:18.151  RC     yaw 1500 -> 1600
    01:31:22.877  =====  END mission 01 forwardTurnBack: OK (12.5 s)  =====

Sources:
  /rosout              every log line of every node (get_logger().info(...) etc.),
                       level >= min_level (default INFO), minus ignore_regex
  /mavros/state        connected / armed / mode, only when they change
  /mavros/rc/override  PWM per axis, only when it changes
  /bluespark/timeline  markers (std_msgs/String), e.g. mission start / end
                       from mission_runner; written as  =====  text  =====

Each line is flushed right away, so the file is complete even after a crash.
"""

import os
import re
import socket
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from mavros_msgs.msg import OverrideRCIn, State
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from bluespark_bringup.vehicle import AXES, SENSOR_QOS

TIMELINE_TOPIC = "/bluespark/timeline"
TIMELINE_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
# Same as the /rosout publishers (rcl defaults): late joiners get the backlog.
ROSOUT_QOS = QoSProfile(
    depth=1000,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# - state_listener prints /mavros/state every second; the STATE lines already
#   show every change of it.
IGNORE_DEFAULT = r"HP: requesting home position|^Connected: (True|False), Armed: "
# Nodes whose INFO lines are only noise here (rosbag lists every topic it
# subscribes to): only their WARN / ERROR go to the timeline.
QUIET_NODES = ("rosbag2_recorder", "rosbag2_cpp")
LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


def fmt_time(stamp_sec=None):
    t = datetime.now() if stamp_sec is None else datetime.fromtimestamp(stamp_sec)
    return t.strftime("%H:%M:%S.%f")[:-3]


class TimelineRecorder(Node):
    def __init__(self):
        super().__init__("timeline_recorder")
        log_dir = self.declare_parameter("log_dir", "").value
        self.min_level = self.declare_parameter("min_level", 20).value
        # Log lines (regex on the message) that only add noise, e.g. mavros asking
        # every 10 s for a home position the vehicle never has (no GPS underwater).
        self.ignore = re.compile(
            self.declare_parameter("ignore_regex", IGNORE_DEFAULT).value or "(?!)"
        )
        if not log_dir:
            raise RuntimeError("timeline_recorder needs the log_dir parameter")
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(log_dir, "timeline.log")
        self.file = open(path, "a", buffering=1, encoding="utf-8")  # line-buffered
        owner = os.stat(log_dir)
        try:  # the container runs as root; keep the file deletable by the Pi user
            os.chown(path, owner.st_uid, owner.st_gid)
        except OSError:
            pass

        self.state = None
        self.rc = None

        self.write_marker(
            f"SESSION START {datetime.now():%Y-%m-%d %H:%M:%S} on {socket.gethostname()}"
        )
        self.create_subscription(Log, "/rosout", self.on_rosout, ROSOUT_QOS)
        self.create_subscription(State, "/mavros/state", self.on_state, SENSOR_QOS)
        self.create_subscription(OverrideRCIn, "/mavros/rc/override", self.on_rc, SENSOR_QOS)
        self.create_subscription(String, TIMELINE_TOPIC, self.on_marker, TIMELINE_QOS)
        self.get_logger().info(f"Writing the session timeline to {path}")

    # --- output ---
    def write(self, kind, text, stamp_sec=None):
        self.file.write(f"{fmt_time(stamp_sec)}  {kind:<6} {text}\n")

    def write_marker(self, text):
        self.write("=====", f"{text}  =====")

    # --- sources ---
    def on_rosout(self, msg):
        if msg.level < self.min_level or msg.name == self.get_name():
            return
        if self.ignore.search(msg.msg):
            return
        if msg.name in QUIET_NODES and msg.level < 30:
            return
        # The time the line was logged (msg.stamp), not when it arrived here:
        # the first lines of a new process arrive up to a few seconds late
        # (discovery). Lines are written in arrival order, so around a process
        # start a line can appear a little after a later one.
        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        level = LEVELS.get(msg.level, str(msg.level))
        self.write(level, f"[{msg.name}] {msg.msg}", stamp)

    def on_state(self, msg):
        old = self.state
        self.state = msg
        if old is None:
            self.write(
                "STATE",
                f"connected={msg.connected} armed={msg.armed} mode={msg.mode}",
            )
            return
        if msg.connected != old.connected:
            self.write("STATE", "Pixhawk CONNECTED" if msg.connected else "Pixhawk DISCONNECTED")
        if msg.armed != old.armed:
            self.write("STATE", "ARMED" if msg.armed else "DISARMED")
        if msg.mode != old.mode:
            self.write("STATE", f"mode {old.mode} -> {msg.mode}")

    def on_rc(self, msg):
        old = self.rc
        self.rc = list(msg.channels[: len(AXES)])
        if old is None:
            self.write("RC", "  ".join(f"{a} {self._pwm(v)}" for a, v in zip(AXES, self.rc)))
            return
        changes = [
            f"{axis} {self._pwm(o)} -> {self._pwm(n)}"
            for axis, o, n in zip(AXES, old, self.rc)
            if o != n
        ]
        if changes:
            self.write("RC", ", ".join(changes))

    @staticmethod
    def _pwm(value):
        if value is None:
            return "?"
        return "released" if value == 0 else str(value)

    def on_marker(self, msg):
        self.write_marker(msg.data)

    def close(self):
        self.write_marker(f"SESSION END {datetime.now():%Y-%m-%d %H:%M:%S}")
        self.file.close()


def main(args=None):
    rclpy.init(args=args)
    node = TimelineRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
