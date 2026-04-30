import rclpy
from docutils.nodes import target
from rclpy.node import Node

from math import hypot
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
        """Initializes all 6 DoFs with stop pwm values and a dirty flag"""
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
        """Sets pwm without sending it immediately"""
        if axis not in self.values: return

        new_pwm = int(new_pwm)

        if self.values[axis] != new_pwm:
            new_pwm = max(1100, min(new_pwm, 1900))
            self.values[axis] = new_pwm
            self.dirty_flags[axis] = True

    def get(self):
        """Gives all axises that have changed and its new pwm values"""
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
    TODO:  In addition it activates depth hold whenever
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
        
        # Creates clients for communcation with MAVROS via our SerRCOverride node
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

        self.stop_everything()

    def stop_and_search(self):
        """ 
        Changes state to searching and starts to rotate
        trying to find a target
        """
        self.current_state = self.STATE_SEARCHING
        self.control_state.set_pwm("surge", STOP_PWM)
        #self.control_state.set_pwm("heave", STOP_PWM)
        self.control_state.set_pwm("yaw", 1550)
        changes = self.control_state.get()
        for axis, pwm in changes.items():
            self.send_rc_override(axis, pwm)

    def send_rc_override(self, axis, pwm_value):
        """ Sents a pwm value to MAVROS via SetRCOverride for given axis """
        self.control_clients[axis].call_async(SetRCOverride.Request(pwm_value=pwm_value))
        self.get_logger().info(f"Sent RC override for {axis} to {pwm_value}")

    def stop_everything(self):
        for axis in self.control_clients.keys():
            self.send_rc_override(axis, STOP_PWM)

    def vision_callback(self, msg):
        """
        Main logic of the node, the P algorithm.
        It is split for states: Centering and approaching
        and we do only one at the time
        """
        # TODO: Aktualnie czeka aż zoabczy osobę i jak zoaczy to do niej płynie
        # TODO: Zrobić żeby płynął do bramki i potem jeszcze nakierowywał na jej środek 

        if len(msg.objects) == 0:
            #self.get_logger().info("No objects detected")
            self.stop_and_search()
            return

        target = None
        for obj in msg.objects:
            if obj.label == "person":
                target = obj
                break
        
        if target is None:
            self.get_logger().info("No person detected among objects")
            self.stop_and_search()
            return

        TARGET_DISTANCE = 1.0
        DEADBAND_ANGLE = 10.0

        # TODO: tweak P parameters
        Kp_yaw = 4.0
        Kp_heave = 4.0
        Kp_surge = 100.0
        Kp_pitch = 4.0
        Kp_roll = 4.0
        Kp_sway = 4.0

        error_yaw = target.cam_h_angle_deg
        error_heave = target.cam_v_angle_deg

        curr_dist = hypot(target.pos_x, target.pos_y, target.pos_z)
        error_dist = curr_dist - TARGET_DISTANCE

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

            # TODO: Decide weather to always approach a bit or not

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
