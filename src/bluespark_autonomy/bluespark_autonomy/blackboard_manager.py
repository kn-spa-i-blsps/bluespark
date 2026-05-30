import py_trees
from bluespark_interfaces.msg import DetectedObjectArray


# TODO: Import the specific message type for depth data when available in bluespark_navigation
# from std_msgs.msg import Float32

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

        # 1. CLIENT CREATION
        # Create a client agent with permissions to modify the blackboard.
        self.bb_client = py_trees.blackboard.Client(name="MasterManager")

        # 2. KEY REGISTRATION (WRITE ACCESS)
        # Register variables before using them to enforce strict architecture.
        self.bb_client.register_key(
            key="vision/detected_objects",
            access=py_trees.common.Access.WRITE
        )

        # TODO: Register depth key once the navigation node provides it
        # self.bb_client.register_key(key="navigation/current_depth", access=py_trees.common.Access.WRITE)

        # 3. INITIALIZATION
        # Set default values so the tree doesn't throw KeyErrors before the first ROS message arrives.
        self.bb_client.set("vision/detected_objects", {})
        # self.bb_client.set("navigation/current_depth", 0.0)

        # 4. ROS 2 SUBSCRIPTIONS
        # TODO: Verify if '/vision/detected_objects' is the exact topic name
        self.vision_sub = self.node.create_subscription(
            DetectedObjectArray,
            '/vision/detected_objects',
            self._vision_callback,
            10
        )

    def _vision_callback(self, msg):
        """
        Callback triggered by new camera frames. Immediately updates the Blackboard.
        """
        # Transform the raw ROS array into a Python dictionary for O(1) lookups in the tree
        # Example: {'Gate': <DetectedObject>, 'Pole': <DetectedObject>}
        detected_dict = {obj.name: obj for obj in msg.objects}

        # 5. WRITE TO BLACKBOARD
        self.bb_client.set("vision/detected_objects", detected_dict)