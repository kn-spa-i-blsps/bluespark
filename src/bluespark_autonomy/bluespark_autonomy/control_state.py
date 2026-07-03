class ControlState:
    """
    Class to store the current state of the control system.
    Maintains safe bounds for PWM and provides full state for keep-alive RC streams.
    """
    STOP_PWM = 1500

    def __init__(self):
        self.values = {
            "pitch" : self.STOP_PWM,
            "roll" : self.STOP_PWM,
            "yaw" : self.STOP_PWM,
            "heave" : self.STOP_PWM,
            "surge" : self.STOP_PWM,
            "sway" : self.STOP_PWM
        }

    def set_pwm(self, axis, new_pwm):
        """Sets pwm value with hard limits (1100-1900)"""
        if axis in self.values:
            self.values[axis] = max(1100, min(int(new_pwm), 1900))

    def get(self):
        """Returns the full dictionary of all 6 axes to prevent RC Override timeouts"""
        return self.values