import py_trees
from bluespark_interfaces.msg import DetectedObjectArray
from mavros_msgs.msg import State, OverrideRCIn, VfrHud
from sensor_msgs.msg import FluidPressure

# TODO: Import the specific message type for depth data when available in bluespark_navigation
class BlackboardManager:
    """
    Purpose:
        The primary memory (Blackboard) manager for the entire behavior tree.
        This is the ONLY class permitted to have WRITE access for sensor data.
        It acts as a bridge between asynchronous ROS 2 callbacks and the synchronous
        py_trees execution, preventing race conditions.

    Usage:
        Instantiate this class once in the main autonomy node before creating the tree.
        Pass the main ROS 2 node reference to it.
        Example:
            bb_manager = BlackboardManager(node)
    """

    def __init__(self, node):
        self.node = node

        ### Vision Blackboard
        self.vision_bb = py_trees.blackboard.Client(name="VisionManager", namespace="vision")
        self.vision_bb.register_key(
            key="detected_objects",
            access=py_trees.common.Access.WRITE
        )
        self.vision_bb.detected_objects = {}
        self.vision_sub = self.node.create_subscription(
            DetectedObjectArray,
            '/front/detected_objects',
            self._vision_callback,
            10
        )

        ### Mavros Blackboard
        self.state_bb = py_trees.blackboard.Client(name="StateBridge", namespace="state")
        self.state_bb.register_key(
            key="is_armed",
            access=py_trees.common.Access.WRITE
        )
        self.state_bb.register_key(
            key="flight_mode",
            access=py_trees.common.Access.WRITE
        )

        self.state_bb.is_armed = False
        self.state_bb.flight_mode = "UNKNOWN"
        self.mavros_sub = self.node.create_subscription(
            State,
            '/mavros/state',
            self._mavros_callback,
            10
        )

        ### DepthBlackboard (Negative altitude: -10m = 10m underwater)
        self.depth_bb = py_trees.blackboard.Client(name="DepthManager", namespace="depth")
        self.depth_bb.register_key(
            key="current_depth",
            access=py_trees.common.Access.WRITE
        )
        self.depth_bb.current_depth = 0.0
        self.atm_pressure = 101325
        self.current_pressure = 101325

        self.depth_sub = self.node.create_subscription(
            FluidPressure,
            'mavros/imu/static_pressure',
            self._depth_callback,
            10
        )


        ###OrbitingBlackboard (azimuth)
        self.orbiting_bb = py_trees.blackboard.Client(name="OrbitingManager", namespace="orbiting")
        self.orbiting_bb.azimuth = 0 #TODO: verify if its neccessary
        self.orbiting_bb.register_key(
            key="azimuth",
            access=py_trees.common.Access.WRITE
        )
        self.orbiting_bb.azimuth = 0 #TODO: verify if its neccessary
        self.orbiting_bb.set("azimuth", 0) #TODO: verify if this doesnt cause any errors
        self.current_azimuth = self.node.create_subsctription(
            VfrHud,
            '/mavros/vfr_hud',
            self._orientation_callback,
            10
        )

        ### ControlBlackboard
        self.control_bb = py_trees.blackboard.Client(name="ControlManager", namespace="control")
        for axis in ["pitch", "roll", "heave", "yaw", "surge", "sway"]:
            self.control_bb.register_key(
                key=f"current_{axis}",
                access=py_trees.common.Access.WRITE
            )
            self.control_bb.set(f"current_{axis}", 1500)

        self.rc_sub = self.node.create_subscription(
            OverrideRCIn,
            '/mavros/rc/override',
            self._rc_override_callback,
            10
        )

    def _rc_override_callback(self, msg):
        if len(msg.channels) >= 6:
            self.control_bb.current_pitch = msg.channels[0]
            self.control_bb.current_roll = msg.channels[1]
            self.control_bb.current_heave = msg.channels[2]
            self.control_bb.current_yaw = msg.channels[3]
            self.control_bb.current_surge = msg.channels[4]
            self.control_bb.current_sway = msg.channels[5]


    def _depth_callback(self, msg):
        self.current_pressure = msg.fluid_pressure
        self.depth_bb.current_depth = (
            (self.atm_pressure - self.current_pressure)/(1000*9.81)
        )

    def _vision_callback(self, msg):
        detected_dict = {obj.name: obj for obj in msg.objects}
        self.vision_bb.set("front/detected_objects", detected_dict)

    def _mavros_callback(self, msg):
        self.state_bb.set("is_armed", msg.armed)
        self.state_bb.set("flight_mode", msg.mode)


    def _orientation_callback(self, msg):
        self.orbiting_bb.azimuth = msg.heading