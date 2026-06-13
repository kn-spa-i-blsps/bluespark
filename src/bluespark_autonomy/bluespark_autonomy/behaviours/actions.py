class ApproachGate(py_trees):
    """
    Action that does a 'step' towards a gate till being in
    """

    def __init__(self, name: str, node):
        super().__init__(name=name)
        self.node = node

        self.blackboard = self.attach_blackboard_client(name=self.name)

        self.blackboard.register_key(
            key="detected_objects",
            access=py_trees.common.Access.READ
        )

    def update(self):