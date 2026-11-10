import time
from .driver import PCA9685Driver
from ..utils.types import PayloadAction

CHANNEL_DROP_BALL_1 = 12
CHANNEL_DROP_BALL_2 = 13
CHANNEL_TORPEDO = 14


class PayloadController:
    def __init__(self, driver: PCA9685Driver):
        self._driver = driver
        self._ball_1 = driver.get_servo(CHANNEL_DROP_BALL_1)
        self._ball_2 = driver.get_servo(CHANNEL_DROP_BALL_2)
        self._torpedo = driver.get_servo(CHANNEL_TORPEDO)

    def drop_ball(self, ball: int = 1):
        if ball == 1:
            servo = self._ball_1
        elif ball == 2:
            servo = self._ball_2
        else:
            return False

        if servo is None:
            return False

        servo.angle = 90
        time.sleep(0.5)
        servo.angle = 0
        return True

    def fire_torpedo(self):
        if self._torpedo is None:
            return False

        self._torpedo.angle = 90
        time.sleep(0.5)
        self._torpedo.angle = 0
        return True

    def reset(self):
        if self._ball_1 is not None:
            self._ball_1.angle = 0
        if self._ball_2 is not None:
            self._ball_2.angle = 0
        if self._torpedo is not None:
            self._torpedo.angle = 0
        return True

    def execute(self, action: PayloadAction):
        if action == PayloadAction.DROP_BALL_1:
            return self.drop_ball(1)
        elif action == PayloadAction.DROP_BALL_2:
            return self.drop_ball(2)
        elif action == PayloadAction.FIRE_TORPEDO:
            return self.fire_torpedo()
        elif action == PayloadAction.RESET:
            return self.reset()
        else:
            return False