try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo
    _HW_AVAILABLE = True
except (ImportError, NotImplementedError, ValueError) as e:
    print(f"HW import failed: {e}")
    _HW_AVAILABLE = False

MIN_PULSE = 800
MAX_PULSE = 2200


class PCA9685Driver:
    def __init__(self, i2c_address: int = 0x40, frequency_hz: int = 50):
        if not _HW_AVAILABLE:
            print("Hardware not available, running in simulation mode")
            self._pca = None
            return
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self._pca = PCA9685(i2c, address=i2c_address)
            self._pca.frequency = frequency_hz
        except Exception as e:
            print(f"I2C init failed: {e}")
            self._pca = None

    def get_servo(self, channel: int):
        if self._pca is None:
            return None
        return servo.Servo(
            self._pca.channels[channel],
            min_pulse=MIN_PULSE,
            max_pulse=MAX_PULSE,
        )
