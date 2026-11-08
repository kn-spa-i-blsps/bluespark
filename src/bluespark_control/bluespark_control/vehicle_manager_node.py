import signal

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import SetBool

"""
Arming, disarming and mode setting needed to use rc_override node.
Goal is to perform arming and mode setting as safely as it is possible
with wide event and error handling.

To use call service:
- State     -> /manager/setmode with desired mode
- Arming    ->  /manager/set_armig with boolean of sedire arming state
"""

# TODO:
# - /manager/set_arming and /manager/set_mode always return success, even if mavros
#   rejects the command. Either wait for the mavros response (needs a
#   MultiThreadedExecutor) or make ArmRobot / SetFlightMode in the tree check
#   state/is_armed and state/flight_mode on the blackboard.
# - Ctrl+C does not disarm: SIGINT raises ExternalShutdownException, which is not
#   caught, so try_to_arm(False) never runs. Same fix as in rc_override_node.
#   (In prod run_governer.sh disarms before stopping the nodes; manual/bench runs
#   have no such safety net.)
# - The three nodes in this process get the same name from the launch file
#   (name='vehicle_manager_node'). Remove name= from the launch or merge them into one node.
# - StateListener logs every /mavros/state message; log only on change.
# - Decide if this proxy is needed at all, or the tree should call mavros directly.


class StateListener(Node):
    """Node for controlling the state of the robot."""

    def __init__(self):
        super().__init__("state_listener")

        self.current_state = State()

        # Subscription of /mavros/state: connected, armed, mode
        self.stat_subscirber = self.create_subscription(
            State, "/mavros/state", self.state_callback, 10
        )

    def state_callback(self, msg):
        self.current_state = msg
        # Publishing received state of the robot
        self.get_logger().info(
            f"Connected: {msg.connected}, Armed: {msg.armed}, Mode: {msg.mode}"
        )


class ArmingNode(Node):
    """
    Creating /mavros/set_arming services which takes true or false.
    """

    def __init__(self):
        super().__init__("arming_node")

        """Client for /mavros/cmd/arming"""
        self.arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")

        self.arm_service = self.create_service(
            SetBool, "/manager/set_arming", self.handle_arm_request
        )
        self.get_logger().info("Arm service ready to take orders.")

    def try_to_arm(self, state: bool) -> bool:
        """Tries to call arming service and passing on the response"""
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Arming service unreachable.")
            return False

        req = CommandBool.Request()
        req.value = state

        future = self.arm_client.call_async(req)

        future.add_done_callback(self.arm_response_callback)
        return True

    def arm_response_callback(self, future):
        """Handling the response of arming service, printing logs"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Command received.")
            else:
                self.get_logger().info("Command declined.")
        except Exception as e:
            self.get_logger().error(f"Service call threw error: {e}")

    def handle_arm_request(self, request, response):
        """Printing the logs while trying to arm"""
        requested_state = request.data

        if requested_state:
            self.get_logger().info("Calling the arming service...")
        else:
            self.get_logger().info("Calling the disarming service...")

        self.try_to_arm(requested_state)

        response.success = True
        response.message = f"Command passed to mavros."

        return response

    # TODO: this code only controls if command was passed to serive or not. It doesnt actualy check if arming was succesful.
    # This needs to be done.


class SetModeNode(Node):
    """
    Creating setting mode node, which takes modes in CAPITAL LETTERS.
    """

    def __init__(self):
        super().__init__("set_manual_mode")

        # creating the client for setting mode service
        self.setmode_client = self.create_client(SetMode, "/mavros/set_mode")

        # creating the set_mode service
        self.setmode_service = self.create_service(
            SetMode, "/manager/set_mode", self.handle_setmode_request
        )

    def set_flight_mode(self, mode_name: str = "GUIDED"):
        """Trying to set mode using custom client, deafult mode: GUIDED"""
        if not self.setmode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Setmode service unreachable.")
            return

        req = SetMode.Request()

        req.custom_mode = mode_name

        future = self.setmode_client.call_async(req)

        future.add_done_callback(self.setmode_response_callback)

    def setmode_response_callback(self, future):
        """Handling the response of set_mode service, printing logs"""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("Command receieved.")
            else:
                self.get_logger().info("Command declined.")
        except Exception as e:
            self.get_logger().error(f"Service call threw error: {e}")

    def handle_setmode_request(self, request, response):
        """Printing the logs while trying to set mode"""
        requested_mode = request.custom_mode

        self.get_logger().info("Request collected. Seting mode...")

        self.set_flight_mode(requested_mode)

        response.mode_sent = True

        return response

    # TODO: this code only controls if command was passed to service or not. It doesnt actualy check if mode setting was succesful.
    # This needs to be done.


def main(args=None):
    rclpy.init(args=args)
    mode_node = SetModeNode()
    arming_node = ArmingNode()
    state_listener_node = StateListener()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(mode_node)
    executor.add_node(arming_node)
    executor.add_node(state_listener_node)

    def handle_sigterm(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, handle_sigterm)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n[Vehicle_Manager] Closing, disarming robot...")
        arming_node.try_to_arm(False)

        executor.spin_once(timeout_sec=0.5)

    finally:
        mode_node.destroy_node()
        arming_node.destroy_node()
        state_listener_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
