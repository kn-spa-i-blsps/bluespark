import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class PathMarkerDetection:
    detected: bool
    offset_x: float        # [-1.0, 1.0] odchylenie od centrum
    angle_deg: float       # [-90, 90] kąt markera względem osi pionowej
    confidence: float      # [0.0, 1.0]
    area_fraction: float   # jak duża część kadru zajmuje marker
    tip_direction: Optional[float]  # kąt wskazywania czubka markera [0, 360]


class PathMarkerDetector:
    def __init__(
        self,
        hsv_lower: tuple = (10, 100, 100),
        hsv_upper: tuple = (25, 255, 255),
        min_area_fraction: float = 0.005,
        min_aspect_ratio: float = 2.0,
    ):
        self.hsv_lower = np.array(hsv_lower)
        self.hsv_upper = np.array(hsv_upper)
        self.min_area_fraction = min_area_fraction
        self.min_aspect_ratio = min_aspect_ratio

    def detect(self, frame: np.ndarray) -> PathMarkerDetection:
        null = PathMarkerDetection(
            detected=False,
            offset_x=0.0,
            angle_deg=0.0,
            confidence=0.0,
            area_fraction=0.0,
            tip_direction=None,
        )

        if frame is None or frame.size == 0:
            return null

        h, w = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return null

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        area_fraction = area / (w * h)

        if area_fraction < self.min_area_fraction:
            return null

        # Dopasuj prostokąt obrócony — daje kąt i proporcje
        rect = cv2.minAreaRect(largest)
        (cx, cy), (rw, rh), angle = rect

        # Upewnij się że dłuższy bok to szerokość
        if rw < rh:
            rw, rh = rh, rw
            angle += 90

        aspect_ratio = rw / (rh + 1e-6)
        if aspect_ratio < self.min_aspect_ratio:
            return null

        offset_x = (cx - w / 2.0) / (w / 2.0)
        offset_y = (cy - h / 2.0) / (h / 2.0)

        # Normalizacja kąta do [-90, 90]
        if angle > 90:
            angle -= 180
        elif angle < -90:
            angle += 180

        confidence = min(area_fraction / 0.05, 1.0)

        # Kierunek czubka — przez moment kontura
        tip_direction = self._detect_tip_direction(largest, cx, cy)

        return PathMarkerDetection(
            detected=True,
            offset_x=float(offset_x),
            angle_deg=float(angle),
            confidence=float(confidence),
            area_fraction=float(area_fraction),
            tip_direction=tip_direction,
        )

    def _detect_tip_direction(
        self,
        contour: np.ndarray,
        cx: float,
        cy: float,
    ) -> Optional[float]:
        """
        Wykrywa w którą stronę wskazuje czubek markera.
        Path marker ma kształt strzałki — jeden koniec jest ostrzejszy.
        Używa momentów kontura do znalezienia asymetrii masy.
        Zwraca kąt w stopniach [0, 360], 0 = góra, 90 = prawo.
        """
        if len(contour) < 5:
            return None

        M = cv2.moments(contour)
        if M['m00'] == 0:
            return None

        # Centroid przez momenty
        mcx = M['m10'] / M['m00']
        mcy = M['m01'] / M['m00']

        # Wektor od geometrycznego centrum do centroidu masy
        dx = mcx - cx
        dy = mcy - cy

        if abs(dx) < 1 and abs(dy) < 1:
            return None

        angle_rad = np.arctan2(dy, dx)
        angle_deg = float(np.degrees(angle_rad))
        # Konwersja do [0, 360], 0 = prawo, 90 = dół
        angle_deg = (angle_deg + 360) % 360

        return angle_deg