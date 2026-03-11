import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

"""

Arming, disarming and mode setting needed use rc_override node.
Goal is to perform arming and mode setting as safely as it is posibble
with wide event and error handling.

"""

#TODO dodać kurwa launchfile'a.

class StateListener(Node):
    def __init__(self):
        super().__init__('state_listener')

        self.current_state = State()

        self.stat_subscirber = self.create_subscription(
            State,
            "/mavros/state",
            self.state_callback,
            10
        )


    def state_callback(self, msg):
        self.current_state = msg

        self.get_logger().info(f"Połączony: {msg.connected}, Uzbrojony: {msg.armed}, Tryb: {msg.mode}")


class ArmingNode(Node):
    def __init__(self):
        super().__init__('arming_node')

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')


    def try_to_arm(self, state: bool):
        if not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Serwis armowania niedostępny.')
            return

        req = CommandBool.Request()
        req.value = state

        future = self.arm_client.call_async(req)

        future.add_done_callback(self.arm_response_callback)

    
    def arm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Komenda przyjęta przez drona.")
            else:
                self.get_logger().info("Komenda nie przyjęta przez drona.")
        except Exception as e:
            self.get_logger().error(f"Wywołanie serwisu wyrzuciło błąd: {e}")


class SetModeManual(Node):
    def __init__(self):
        super().__init__('set_manual_mode')

        self.setmode_client = self.create_client(SetMode, '/mavros/set_mode')

    
    def set_flight_mode(self, mode_name: str):
        if not self.setmode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Serwis setmode niedostępny.')
            return
        
        req = SetMode.Request()

        req.custom_mode = mode_name

        future = self.setmode_client.call_async(req)

        future.add_done_callback(self.setmode_response_callback)
    
    def setmode_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("Komenda przyjęta przez drona.")
            else:
                self.get_logger().info("Komenda nie przyjęta przez drona.")
        except Exception as e:
            self.get_logger().error(f"Wywołanie serwisu wyrzuciło błąd: {e}")


def main(args=None):
    rclpy.init(args=args)
    mode_node = SetModeManual()
    arming_node = ArmingNode()
    state_listener_node = StateListener()
    

    executor = rclpy.executors.SingleThreadedExecutor()

    executor.add_node(mode_node)
    executor.add_node(arming_node)
    executor.add_node(state_listener_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        mode_node.destroy_node()
        arming_node.destroy_node()
        state_listener_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()