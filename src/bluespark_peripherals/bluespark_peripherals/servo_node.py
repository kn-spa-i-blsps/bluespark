import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from bluespark_peripherals.utils.types import PayloadAction
from bluespark_peripherals.servo.driver import PCA9685Driver
from bluespark_peripherals.servo.payload import PayloadController

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        self._driver = PCA9685Driver()
        self._controller = PayloadController(self._driver)

        self._subscription = self.create_subscription(
            Int32,
            '/payload/action',
            self._on_action, 
            10,

        )

        self.get_logger().info("servo node ready, listening on /payload/action")    

    def _on_action(self, msg: Int32):
        try: 
            action=PayloadAction(msg.data)
        except ValueError:
            self.get_logger().error(f"Invalid action value: {msg.data}")
            return
        self.get_logger().info(f"Received action: {action.name}")
        success=self._controller.execute(action)

        if success:
            self.get_logger().info(f"Action {action.name} executed successfully.")
        else:
            self.get_logger().error(f"Action {action.name} failed to execute.")



def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()