"""
Node for giving power to thrusters with raw pwm, which is earlier computed with pid controller
Used by everything wanting to use thruster e.g. autonomy pipeline

To use call service: control/set_[degree of freedom] with given PWM value between 1100-1900 with 1500 meaning STOP
"""

# TODO:
# - Dead man switch: subscribe to a heartbeat from the behaviour tree and set all
#   axes to 1500 if nothing arrives for ~1 s. Arm the watchdog only after the first
#   heartbeat, so manual service calls from the CLI still work.
# - Ctrl+C does not stop the thrusters: rclpy handles SIGINT and shuts down the
#   context before `finally`, so the 1500 publish fails. Use
#   rclpy.init(signal_handler_options=SignalHandlerOptions.NO) and handle
#   SIGINT/SIGTERM ourselves.
# - Clamp pwm_value to 1100-1900 in the service callbacks (0 = release is allowed).
# - Confirm RCMAP_* params in QGC match the channel mapping below, then drop this note.
# - Confirm the heave direction (> 1500 up or down?), comments in the repo disagree.

import os
import signal
import time

import rclpy
from mavros_msgs.msg import OverrideRCIn
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from bluespark_interfaces.srv import SetRCOverride


# TODO: add dead man swithc to avoid giving power to engines if exception is thrown
class RCOverrideNode(Node):
    """Node that holds arrays of values on all 18 RC channels and sends them"""

    def __init__(self):
        super().__init__("rc_override_node")

        self.rc_channels = [0] * 18

        self.rc_pub = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)

        self.timer = self.create_timer(0.1, self.publish_rc_state)

        """
        NAMES OF THE SERVICES, PREPARED TO BE CALLED BY HIGHER-LEVEL DECISIONS SCRIPT
        """
        self.srv_pitch = self.create_service(
            SetRCOverride, "control/set_pitch", self.cb_pitch
        )
        self.srv_roll = self.create_service(
            SetRCOverride, "control/set_roll", self.cb_roll
        )
        self.srv_heave = self.create_service(
            SetRCOverride, "control/set_heave", self.cb_heave
        )
        self.srv_yaw = self.create_service(
            SetRCOverride, "control/set_yaw", self.cb_yaw
        )
        self.srv_surge = self.create_service(
            SetRCOverride, "control/set_surge", self.cb_surge
        )
        self.srv_sway = self.create_service(
            SetRCOverride, "control/set_sway", self.cb_sway
        )

    def publish_rc_state(self):
        msg = OverrideRCIn()
        msg.channels = self.rc_channels
        self.rc_pub.publish(msg)

    """
    Callbacks of services
    Notice: the callbacks dont actually send any values to the robot
    They only modify values in the array in the memory.
    In fact, the function above (publish_rc_state) combined with
    self.rc_pub and self.timer are resposible for passing the values
    to the robot.
    """

    def cb_pitch(self, request, response):
        self.rc_channels[0] = request.pwm_value
        response.success = True
        return response

    def cb_roll(self, request, response):
        self.rc_channels[1] = request.pwm_value
        response.success = True
        return response

    def cb_heave(self, request, response):
        self.rc_channels[2] = request.pwm_value
        response.success = True
        return response

    def cb_yaw(self, request, response):
        self.rc_channels[3] = request.pwm_value
        response.success = True
        return response

    def cb_surge(self, request, response):
        self.rc_channels[4] = request.pwm_value
        response.success = True
        return response

    def cb_sway(self, request, response):
        self.rc_channels[5] = request.pwm_value
        response.success = True
        return response

    # OTHER CHANNELS ARE 0


def main(args=None):
    rclpy.init(args=args)
    node = RCOverrideNode()

    def handle_sigterm(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, handle_sigterm)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        print("\n[RC Override] Stopping engines (sending PWM 1500)...")

        node.rc_channels = [1500, 1500, 1500, 1500, 1500, 1500] + [0] * 12
        node.publish_rc_state()

        time.sleep(0.1)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
