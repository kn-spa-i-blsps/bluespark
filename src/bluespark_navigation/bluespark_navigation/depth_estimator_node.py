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
    def __init__(self):
        super().__init__("depth_estimator_node")

        self.atm_pressure = 101325
        self.current_pressure = 101325
        self.depth = 0


        self.depth_sub = self.create_subscription(
            FluidPressure,
            'mavros/imu/static_pressure',
            self.estimator_callback,
            10
        )


        self.depth_pub = self.create_publisher(
            Float64,
            #TEMAT NA KTÓRYM PUBLIKUJE
            '/current_depth',
            10
        )

        self.get_logger().info('Depth estimator gotowy do pracy')

    def estimator_callback(self, msg):
        self.current_pressure = msg.fluid_pressure
        # liczenie z użyciem ciśnienia hydrostatycznego słupa cieczy
        self.current_depth = (
            (self.atm_pressure - self.current_depth)/(1000*9.81)
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
