import math
import json
import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os

class SimpleDistanceCalculator:

    # init
    def __init__(self, calibration_file=None):
        pkg_share_dir = get_package_share_directory('bluespark_vision')
        if calibration_file is None:
            calibration_file = os.path.join(pkg_share_dir, 'calibration_files', 'camera_calibration.json')

        self.camera_matrix, self.dist_coeffs = self.load_calibration(
            calibration_file)

        objects_file = os.path.join(pkg_share_dir, 'calibration_files', 'object_config.json')
        self.object_attrs = self.load_object_config(objects_file)

    # load objects configuration
    def load_object_config(self, config_file):
        try:
            with open(config_file, "r") as f:
                config = json.load(f)
                return config["objects"]
        except FileNotFoundError:
            print(f"ERROR: file not found: {config_file}")
            return {}
        except KeyError:
            print(f"Error: json file must contain key 'objects'.")
            return {}

    # load camera calibration information
    def load_calibration(self, calibration_file):
        try:
            with open(calibration_file, "r") as f:
                calib = json.load(f)
            camera_matrix = np.array(calib["camera_matrix"])
            dist_coeffs = np.array(calib["dist_coeffs"])
            return camera_matrix, dist_coeffs
        except FileNotFoundError:
            print("Warning: Calibration file not found, using defaults.")
            # default matrix for generic webcam (approximate)
            return np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]]), np.zeros(5)

    # calculate distance, position and object rotation
    def calculate_pose(self, bbox, label_name):
        """
        Calculates distance, global angles and object rotation using bounding box.
        """
        if label_name not in self.object_attrs:
            return None

        # camera parameters
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        ppx = self.camera_matrix[0, 2]
        ppy = self.camera_matrix[1, 2]

        x1, y1, x2, y2 = bbox

        obj_width_px = x2 - x1
        obj_height_px = y2 - y1
        obj_center_x = (x2 + x1) / 2
        obj_center_y = (y2 + y1) / 2

        real_h = self.object_attrs[label_name]["real_height"]
        real_w = self.object_attrs[label_name]["real_width"]

        pos_z = real_h * fy / obj_height_px

        pos_x = (obj_center_x - ppx) * pos_z / fx
        pos_y = (obj_center_y - ppy) * pos_z / fy

        cam_h_angle = math.atan2(pos_x, pos_z) * (180.0 / math.pi)
        cam_v_angle = math.atan2(pos_y, pos_z) * (180.0 / math.pi)

        real_ratio = real_w / real_h
        
        obs_ratio = obj_width_px / obj_height_px
        
        ratio_factor = obs_ratio / real_ratio

        if ratio_factor > 1.0: 
            ratio_factor = 1.0
        if ratio_factor < -1.0:
            ratio_factor = -1.0
            
        obj_rotation_rad = math.acos(ratio_factor)
        obj_rotation_deg = obj_rotation_rad * (180.0 / math.pi)

        return {
            "pos": (pos_x, pos_y, pos_z),
            "cam_angles": (cam_h_angle, cam_v_angle),
            "obj_rotation": obj_rotation_deg,
            "ratios": (real_ratio, obs_ratio),
            "center_px": (obj_center_x, obj_center_y)
        }

    def draw_info(self, frame, bbox, label_name, confidence):
        x1, y1, x2, y2 = bbox
        
        data = self.calculate_pose(bbox, label_name)

        if data is None:
            return

        pos_x, pos_y, pos_z = data["pos"]
        h_angle, v_angle = data["cam_angles"]
        obj_rot = data["obj_rotation"]
        
        color_g = int(255 * (1 - (obj_rot/90.0)))
        color_r = int(255 * (obj_rot/90.0))
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, color_g, color_r), 2)

        dist_text = f"Dist: {pos_z:.2f}m"
        rot_text  = f"Rot: {obj_rot:.0f} deg"
        ratio_text = f"Ratio: {data['ratios'][1]:.2f}/{data['ratios'][0]:.2f}"
        label_text = f"{label_name} ({confidence:.2f})"

        lines = [label_text, dist_text, rot_text, ratio_text]
        
        for i, line in enumerate(lines):
            y_pos = y1 - 10 - (len(lines) - i - 1) * 20
            if y_pos < 10: 
                y_pos = y2 + 20 + i * 20
                
            cv2.putText(frame, line, (x1, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
