import cv2
import time

from .exceptions import CameraError

# TODO check if this is necessary or can be done better
# check for picamera if csi camera is used

try:
    from picamera2 import Picamera2
    PICAM2_AVAILABLE = True
except ImportError:
    PICAM2_AVAILABLE = False

class UniversalCamera:
    """
    Define which camera defice should be used based on user input
    or whether picamera software is available (prefere picamera when 
    no mode (or auto) is selected). Use appropriate camera device.
    """

    def __init__(self, width=640, height=480, mode: str = "auto"):
        self.width = width
        self.height = height
        self.rpi_cam = None
        self.cv_cam = None
        self.mode = mode

        self.init_camera()

    def init_camera(self):
        if self.mode == "tcp":
            self._init_cv_camera("tcp://127.0.0.1:5000")
            return
        use_rpi = False
        if self.mode == "rpi":
            if not PICAM2_AVAILABLE:
                raise CameraError("[ERROR] picamera2 module is not installed.")
            use_rpi = True
        elif self.mode == "usb":
            use_rpi = False
        else: 
            if PICAM2_AVAILABLE:
                print("[AUTO] Picamera2 module found, using CSI camera.")
                use_rpi = True
            else:
                print("[AUTO] Picamera2 module not found, using usb camera.")
                use_rpi = False

        if use_rpi:
            self._init_rpi_camera()
        else:
            self._init_cv_camera(0)
        
    def _init_rpi_camera(self):
        try:
            self.rpi_cam = Picamera2()
            config = self.rpi_cam.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "XBGR8888"}
            )
            self.rpi_cam.configure(config)
            self.rpi_cam.start()

            print("[CAM] Waiting 2s for sensor stabilization...")
            time.sleep(2.0)
            print("[CAM] RPI camera sensor is ready.")
        except Exception as e:
            raise CameraError(f"CSI camera initialization error: {e}.")

    def _init_cv_camera(self, source):
        self.cv_cam = cv2.VideoCapture(source)
        self.cv_cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cv_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if not self.cv_cam.isOpened():
            raise CameraError(f"USB camera cap is not opened.")

    def read(self):
        if self.rpi_cam:
            # convert frame to cvt format for bbox drawing 
            try:
                frame = self.rpi_cam.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                return True, frame
            except:
                return False, None
        elif self.cv_cam:
            return self.cv_cam.read()
        return False, None

    def release(self):
        if self.rpi_cam:
            self.rpi_cam.stop()
            self.rpi_cam.close()
        elif self.cv_cam:
            self.cv_cam.release()

    def isOpened(self): 
        # FIXME do proper check whether piCamera is available    
        if self.rpi_cam: return True 
        return self.cv_cam.isOpened() if self.cv_cam else False
        
    def set(self, propId, value):
        if self.cv_cam: self.cv_cam.set(propId, value)
