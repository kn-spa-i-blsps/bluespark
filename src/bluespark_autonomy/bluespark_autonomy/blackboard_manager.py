import py_trees
from bluespark_interfaces.msg import DetectedObjectArray
from mavros_msgs.msg import State


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
            '/vision/detected_objects',
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

        # TODO: Register depth key once the navigation node provides it and all the other sensors



    def _vision_callback(self, msg):
        detected_dict = {obj.name: obj for obj in msg.objects}
        self.vision_bb.set("vision/detected_objects", detected_dict)

    def _mavros_callback(self, msg):
        self.state_bb.set("is_armed", msg.armed)
        self.state_bb.set("flight_mode", msg.mode)