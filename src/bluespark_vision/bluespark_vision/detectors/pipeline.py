import cv2
import numpy as np
from dataclasses import dataclass


@dataclass
class PipelineDetection:
    detected: bool
    offset_x: float      # [-1.0, 1.0], 0 = centrum obrazu
    angle_deg: float     # [-90, 90], 0 = rurociąg pionowy w kadrze
    confidence: float    # [0.0, 1.0]
    line_width_px: int


class PipelineDetector:
    def __init__(
        self,
        hsv_lower: tuple = (0, 0, 200),
        hsv_upper: tuple = (180, 35, 255),
        hough_threshold: int = 50,
        hough_min_line_length: int = 80,
        hough_max_line_gap: int = 30,
    ):
        self.hsv_lower = np.array(hsv_lower)
        self.hsv_upper = np.array(hsv_upper)
        self.hough_threshold = hough_threshold
        self.hough_min_line_length = hough_min_line_length
        self.hough_max_line_gap = hough_max_line_gap

    def detect(self, frame: np.ndarray) -> PipelineDetection:
        null = PipelineDetection(
            detected=False,
            offset_x=0.0,
            angle_deg=0.0,
            confidence=0.0,
            line_width_px=0,
        )

        if frame is None or frame.size == 0:
            return null

        h, w = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        edges = cv2.Canny(mask, 50, 150)

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=self.hough_threshold,
            minLineLength=self.hough_min_line_length,
            maxLineGap=self.hough_max_line_gap,
        )

        if lines is None:
            return null

        best_line = None
        best_length = 0.0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = np.hypot(x2 - x1, y2 - y1)
            if length > best_length:
                best_length = length
                best_line = line[0]

        if best_line is None:
            return null

        x1, y1, x2, y2 = best_line
        cx = (x1 + x2) / 2.0

        offset_x = (cx - w / 2.0) / (w / 2.0)

        dx = x2 - x1
        dy = y2 - y1
        # Normalizacja kąta do [-90, 90]
        # arctan2(dx, dy) daje kąt względem osi Y
        # fmod + przesunięcie zapewnia że zawsze dostajemy ten sam kąt
        # dla linii pionowej niezależnie od kierunku wektora
        angle_rad = np.arctan2(dx, dy)
        angle_deg = float(np.degrees(angle_rad))
        if angle_deg > 90:
            angle_deg -= 180
        elif angle_deg < -90:
            angle_deg += 180

        diagonal = np.hypot(w, h)
        confidence = float(min(best_length / diagonal, 1.0))

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        line_width_px = 0
        if contours:
            largest = max(contours, key=cv2.contourArea)
            _, _, cw, _ = cv2.boundingRect(largest)
            line_width_px = int(cw)

        return PipelineDetection(
            detected=True,
            offset_x=float(offset_x),
            angle_deg=angle_deg,
            confidence=confidence,
            line_width_px=line_width_px,
        )