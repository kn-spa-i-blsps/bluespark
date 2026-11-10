import numpy as np
from sensor_msgs.msg import Image

# TODO: get rid of magic numbers, comment shit out of this

def cv2_to_ros_image(frame, frame_id="camera_link", stamp=None) -> Image:
    msg = Image()
    if stamp:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = frame.shape[1] * frame.shape[2] 
    msg.data = frame.tobytes()
    return msg

def ros_image_to_cv2(msg: Image) -> np.ndarray:
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)