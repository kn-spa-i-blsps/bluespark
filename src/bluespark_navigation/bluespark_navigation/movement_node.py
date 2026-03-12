import rclpy
from docutils.nodes import target
from rclpy.node import Node

from bluespark_interfaces.msg import DetectedObjectArray
from bluespark_interfaces.srv import SetRCOverride

STOP_PWM = 1500

class ControlState:
    """
    Class to store the current state of the control system.
    That knows all the pwm values and which are already sent
    and which aren't
    """
    def __init__(self):
        self.values = {
            "pitch" : STOP_PWM,
            "roll" : STOP_PWM,
            "yaw" : STOP_PWM,
            "heave" : STOP_PWM,
            "surge" : STOP_PWM,
            "sway" : STOP_PWM
        }
        self.dirty_flags = {axis: False for axis in self.values}

    def set_pwm(self, axis, new_pwm):
        if axis not in self.values: return

        new_pwm = int(new_pwm)

        if self.values[axis] != new_pwm:
            new_pwm = max(1100, min(new_pwm, 1900))
            self.values[axis] = new_pwm
            self.dirty_flags[axis] = True

    def get(self):
        changes = {}
        for axis, is_dirty in self.dirty_flags.items():
            if is_dirty:
                changes[axis] = self.values[axis]
                self.dirty_flags[axis] = False
        return changes



class MovementNode(Node):
    """
    Node responsible for sending pwm values to chosen axis, for
    swimming closer to a specific target.
    """

    def __init__(self):
        super().__init__('movement_node')

        # Subscription to the topic of the vision node
        self.vision_sub = self.create_subscription(
            DetectedObjectArray,
            "/detected_objects",
            self.vision_callback,
            10
        )

        self.control_clients = {
            "pitch": self.create_client(SetRCOverride, 'control/set_pitch'),
            "roll": self.create_client(SetRCOverride, 'control/set_roll'),
            "heave": self.create_client(SetRCOverride, 'control/set_heave'),
            "yaw": self.create_client(SetRCOverride, 'control/set_yaw'),
            "surge": self.create_client(SetRCOverride, 'control/set_surge'),
            "sway": self.create_client(SetRCOverride, 'control/set_sway')
        }

        for client in self.control_clients.keys():
            while not self.control_clients[client].wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for RC override service {client} to become available")

        self.STATE_SEARCHING = "SEARCHING"
        self.STATE_CENTERING = "CENTERING"
        self.STATE_APPROACHING = "APPROACHING"

        self.current_state = self.STATE_SEARCHING
        self.control_state = ControlState()

        self.get_logger().info("Movement node started.")

    def send_rc_override(self, axis, pwm_value):
        self.control_clients[axis].call_async(SetRCOverride.Request(pwm_value=pwm_value))
        self.get_logger().info(f"Sent RC override for {axis} to {pwm_value}")

    def vision_callback(self, msg):
        """
        Main logic of the node, the P algorithm.
        It is split for states: Centering and approaching
        and we do only one at the time
        TODO Narazie zakładamy że widzi obiekt od poćzątku
         i ten który jest tym pierwszy, do tego płynie
        TODO Zrobić żeby płynął do bramki
        """

        if len(msg.objects) == 0: self.logger.info("No objects detected")

        target = msg.objects[0]

        TARGET_DISTANCE = 1.0
        DEADBAND_ANGLE = 5.0

        # TODO tweak P parameters
        Kp_yaw = 4.0
        Kp_pitch = 4.0
        Kp_roll = 4.0
        Kp_heave = 4.0
        Kp_surge = 100.0
        Kp_sway = 4.0

        # TODO sanity check Maciek
        error_yaw = target.cam_h_angle_deg
        error_heave = target.cam_v_angle_deg

        # TODO zmienić na fancy funckje wylicznaia tej odleggłości z 3 zmiennych i ogniskowej
        # TODO 2 ALBO NIE XD
        error_dist = target.pos_z - TARGET_DISTANCE

        is_centered = ((abs(error_yaw) < DEADBAND_ANGLE)
                       and (abs(error_heave) < DEADBAND_ANGLE))

        if not is_centered:
            self.current_state = self.STATE_CENTERING
        else:
            self.current_state = self.STATE_APPROACHING

        # CENTERING
        if self.current_state == self.STATE_CENTERING:
            self.control_state.set_pwm("surge", STOP_PWM)

            # YAWING - TURNING AROUND
            if abs(error_yaw) > DEADBAND_ANGLE:
                yaw_pwm = 1500 + (error_yaw * Kp_yaw)
                self.control_state.set_pwm("yaw", yaw_pwm)
            else:
                self.control_state.set_pwm("yaw", STOP_PWM)

            # HEAVING - UP AND DOWN (DEEPNESS)
            if abs(error_heave) > DEADBAND_ANGLE:
                heave_pwm = 1500 + (error_heave * Kp_heave)
                self.control_state.set_pwm("heave", heave_pwm)
            else:
                self.control_state.set_pwm("heave", 1500)

        # APPROACHING
        elif self.current_state == self.STATE_APPROACHING:
            self.control_state.set_pwm("yaw", STOP_PWM)
            self.control_state.set_pwm("heave", STOP_PWM)

            #SURGING - FORWARD AND BACKWARD
            surge_pwm = 1500 + (error_dist * Kp_surge)
            self.control_state.set_pwm("surge", surge_pwm)

        # Sending changes to RC
        changes = self.control_state.get()
        for axis, pwm in changes.items():
            self.send_rc_override(axis, pwm)



def main(args=None):
    rclpy.init(args=args)
    node = MovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()