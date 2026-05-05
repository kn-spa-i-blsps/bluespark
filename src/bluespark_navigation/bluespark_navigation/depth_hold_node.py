import rclpy
from rclpy.node import Node

from bluespark_interfaces.srv import SetRCOverride
from mavros_msgs.msg import State

from std_msgs.msg import Float64


STOP_PWM = 1500

class DepthHoldNode(Node):
    """
    Proportional depth controller layered on top of ArduSub's ALT_HOLD.

    Conventions:
        depth: surface = 0.0, below surface = NEGATIVE.
        PWM:   1500 = neutral, > 1500 = ascend, < 1500 = descend.
    """

    def __init__(self):
        super().__init__('depth_hold_node')
        # TODO: target_depth from mission planner
        self.target_depth = -0.3

        # band around the setpoint (avoid oscillating on sensor noise)
        self.deadband = 0.1

        # gain: PWM units per meter of error
        self.gain = 200

        # hard PWM limits for safety
        self.heave_up_max = 1600
        self.heave_down_min = 1400

        self.current_depth = 0.0
        self.current_mode = ""

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Float64,
            '/current_depth',
            self.depth_callback,
            10
        )

        self.heave_client = self.create_client(
            SetRCOverride,
            'control/set_heave'
        )

        while not self.heave_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('-- Waiting for control/set_heave service --')

        self.get_logger().info(
            f'Depth hold ready. Target: {self.target_depth}m'
        )

    def state_callback(self, msg):
        self.current_mode = msg.mode

    def depth_callback(self, msg):
        # only active in ALT_HOLD
        if self.current_mode != "ALT_HOLD":
            return

        self.current_depth = msg.data
        error = self.target_depth - self.current_depth

        # inside the deadband: hand the channel back to ALT_HOLD
        if abs(error) < self.deadband:
            self.send_heave(STOP_PWM)
            return

        # scale error to PWM bias
        heave_pwm = STOP_PWM + int(self.gain * error)
        # limit to safe PWM range
        heave_pwm = max(self.heave_down_min, min(self.heave_up_max, heave_pwm))

        self.send_heave(heave_pwm)

        self.get_logger().info(
            f'Depth: {self.current_depth:.2f}m | Target: {self.target_depth:.2f}m | PWM: {heave_pwm}'
        )

    def send_heave(self, pwm_value):
        req = SetRCOverride.Request()
        req.pwm_value = pwm_value
        self.heave_client.call_async(req)

def main(args = None):
    rclpy.init(args = args)
    node = DepthHoldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # release the channel back to the autopilot before shutdown
        node.send_heave(STOP_PWM)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
