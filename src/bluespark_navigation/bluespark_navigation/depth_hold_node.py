import rclpy
from rclpy.node import Node

from bluespark_interfaces.srv import SetRCOverride
from mavros_msgs.msg import State


STOP_PWM = 1500

class DepthHoldNode(Node):
    def __init__(self):
        super().__init__('depth_hold_node')
        # TODO: zmienic na topic od mission plannera
        self.target_depth = -0.7

        self.deadband = 0.1

        self.heave_up = 1550
        self.heave_down = 1450

        self.current_depth = 0.0
        self.current_mode = ""

        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            VfrHud,
            '/current_depth',
            self.depth_callback,
            10
        )

        self.heave_client = self.create_client(
            SetRCOverride,
            'control/set_heave'
        )

        while not self.heave_client.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('-- Czekam na serwis control/set_heave --')

        self.get_logger().info(
            f'Depth hold gotowy. Cel: {self.target_depth}m'
        )

    def state_callback(self, msg):
        self.current_mode = msg.mode

    def depth_callback(self, msg):
        if self.current_mode != "ALT_HOLD":
            return
            
        self.current_depth = msg.data
        error = self.target_depth - self.current_depth

        if abs(error) < self.deadband:
            self.send_heave(STOP_PWM)
            return

        if error < 0:
            heave_pwm = self.heave_down
        else:
            heave_pwm = self.heave_up

        self.send_heave(heave_pwm)

        self.get_logger().info(
            f'Glebokosc: {self.current_depth:.2f}m | Cel: {self.target_depth:.2f}m | PWM: {heave_pwm}'
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
        node.send_heave(STOP_PWM)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

