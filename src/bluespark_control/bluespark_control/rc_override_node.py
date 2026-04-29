import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from bluespark_interfaces.srv import SetRCOverride
import signal
import time
import os

# TODO: implement the service of arming the robot and setting
# it to the manual mode. It has to be called always before the
# RCOverrideNode

class RCOverrideNode(Node):
    def __init__(self):
        super().__init__('rc_override_node')

        self.rc_channels = [0] * 18

        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        self.timer = self.create_timer(0.1, self.publish_rc_state)

        """
        NAMES OF THE SERVICES, PREPARED TO BE CALLED BY HIGHER-LEVEL DECISIONS SCRIPT
        """
        self.srv_pitch = self.create_service(SetRCOverride, 'control/set_pitch', self.cb_pitch)
        self.srv_roll = self.create_service(SetRCOverride, 'control/set_roll', self.cb_roll)
        self.srv_heave = self.create_service(SetRCOverride, 'control/set_heave', self.cb_heave)
        self.srv_yaw = self.create_service(SetRCOverride, 'control/set_yaw', self.cb_yaw)
        self.srv_surge = self.create_service(SetRCOverride, 'control/set_surge', self.cb_surge)
        self.srv_sway = self.create_service(SetRCOverride, 'control/set_sway', self.cb_sway)


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

    
def main(args=None):
    rclpy.init(args=args)
    node = RCOverrideNode()

    def sig_handler(signum, frame):
        print("[RC Override] powering off engines...")
        node.rc_channels = [0] * 18
        node.publish_rc_state()
        
        time.sleep(0.2)
        
        node.destroy_node()
        rclpy.shutdown()
        os.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

