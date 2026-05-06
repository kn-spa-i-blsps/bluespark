import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64

"""
Logika do obliczania głębokości z surowych danych
ciśnienia. Publikuje obliczoną głębokości na topicu
/current_depth.
Nie przetestowałem, bo nie zdążyłem.
Problem do rozwiązania: ciśnienie atmosferyczne
może być różne w różne dni co może powodować rozbieżności
ale niewielkie. Sprawdzić, czy to znaczące.
"""
class DepthEstimatorNode(Node):
    """
    Node responsible for calculating the altitude of AUV from pressure data.
    """
    def __init__(self):
        super().__init__("depth_estimator_node")
        # Initialization of average atmospheric pressure
        self.atm_pressure = 101325
        self.current_pressure = 101325
        self.depth = 0

        """
        Subscription to the raw pressure data topic.
        There is also possiblity to use mavros/vfr_hud, and 
        /mavros/local_position/pose but they are not calibrated
        """
        self.depth_sub = self.create_subscription(
            FluidPressure,
            'mavros/imu/static_pressure',
            self.estimator_callback,
            10
        )

        # Creates a publisher with current depth data
        self.depth_pub = self.create_publisher(
            Float64,
            # Topic which is published
            '/current_depth',
            10
        )

        self.get_logger().info('Depth estimator gotowy do pracy')

    def estimator_callback(self, msg):
        """ Main logic of the node: calculating the altitude using hydrostatic pressure """
        self.current_pressure = msg.fluid_pressure
        self.current_depth = (
            (self.atm_pressure - self.current_pressure)/(1000*9.81)
            # Note that the altitude is NEGATIVE
        )
        depth_msg = Float64()
        depth_msg.data = self.current_depth
        self.depth_pub.publish(depth_msg)





def main(args = None):
    rclpy.init(args=args)
    node = DepthEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    

if __name__ == "__main__":
    main()
