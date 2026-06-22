"""
File: hardcoded_actions.py
Purpose: Contains behavior tree action nodes for open-loop (dead reckoning)
         control and system state management via ROS 2 services.
         These behaviors are entirely independent of the visual perception system
         and are used for hardware validation and blind navigation.
"""

import py_trees
from mavros_msgs.srv import SetMode
from std_srvs.srv import SetBool
import time
from bluespark_interfaces.srv import SetRCOverride
from py_trees.composites import Sequence

# ==============================================================================
# PHASE 1: PRE-FLIGHT SETUP & SYSTEM STATE MANAGEMENT
# ==============================================================================


class MoveRC(py_trees.behaviour.Behaviour):
    """
    Purpose:
        Executes an open-loop movement by sending PWM values (1100-1900) to the
        RCOverrideNode services. Maintains the command for a specified duration,
        then automatically stops the vehicle upon completion or preemption.

    Usage:
        Add to a Sequence node. 1500 means stop.
        Example:
            MoveRC("Move Forward", duration_sec=5.0, surge=1600)
    """

    def __init__(self, name, duration_sec, pitch=1500, roll=1500, heave=1500, yaw=1500, surge=1500, sway=1500):
        super().__init__(name)
        self.duration_sec = duration_sec

        # PWM values for 6 DOF
        self.cmd_values = {
            'pitch': pitch, 'roll': roll, 'heave': heave,
            'yaw': yaw, 'surge': surge, 'sway': sway
        }

        self.clients = {}
        self.start_time = None

    def setup(self, **kwargs):
        try:
            node = kwargs['node']
            # Create clients for all 6 degree-of-freedom services
            self.clients['pitch'] = node.create_client(SetRCOverride, 'control/set_pitch')
            self.clients['roll'] = node.create_client(SetRCOverride, 'control/set_roll')
            self.clients['heave'] = node.create_client(SetRCOverride, 'control/set_heave')
            self.clients['yaw'] = node.create_client(SetRCOverride, 'control/set_yaw')
            self.clients['surge'] = node.create_client(SetRCOverride, 'control/set_surge')
            self.clients['sway'] = node.create_client(SetRCOverride, 'control/set_sway')
        except KeyError:
            self.logger.error("ROS 2 node reference missing in setup()!")

    def _send_commands(self, values_dict):
        """Helper method to send asynchronous requests to all axes."""
        for axis, pwm in values_dict.items():
            if self.clients[axis].service_is_ready():
                req = SetRCOverride.Request()
                req.pwm_value = pwm
                self.clients[axis].call_async(req)
            else:
                self.logger.warning(f"[{self.name}] Service for {axis} is unavailable!")

    def initialise(self):
        self.logger.info(f"[{self.name}] Starting movement for {self.duration_sec}s")
        self.logger.debug(f"[{self.name}] Commands: {self.cmd_values}")

        # Send targeted movement commands and start the timer
        self._send_commands(self.cmd_values)
        self.start_time = time.time()

    def update(self):
        # Check if the requested time has elapsed
        if time.time() - self.start_time >= self.duration_sec:
            self.logger.info(f"[{self.name}] Movement completed.")
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # SAFETY GUARANTEE: Always send 1500 (STOP) to all axes upon exiting the node.
        # This executes on SUCCESS, FAILURE, or if preempted by a higher-level timeout.
        self.logger.debug(f"[{self.name}] Terminating - Sending STOP (1500) to all thrusters.")
        stop_values = {axis: 1500 for axis in self.cmd_values.keys()}
        self._send_commands(stop_values)
