import cv2
import numpy as np
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple


class SymbolDetectionMethod(Enum):
    HSV = "hsv"
    TEMPLATE = "template"


@dataclass
class BinDetection:
    detected: bool
    offset_x: float
    offset_y: float
    area_fraction: float
    symbol: Optional[str]
    symbol_confidence: float
    bbox: Tuple[int, int, int, int]


@dataclass
class BinDetectorConfig:
    bin_hsv_lower: tuple = (100, 80, 30)
    bin_hsv_upper: tuple = (130, 255, 150)
    min_bin_area_fraction: float = 0.01

    symbol_method: SymbolDetectionMethod = SymbolDetectionMethod.HSV

    flame_hsv_lower: tuple = (0, 100, 150)
    flame_hsv_upper: tuple = (25, 255, 255)

    drop_hsv_lower: tuple = (140, 80, 150)
    drop_hsv_upper: tuple = (170, 255, 255)

    flame_template_path: str = ""
    drop_template_path: str = ""
    template_match_threshold: float = 0.7


class BinDetector:
    def __init__(self, config: BinDetectorConfig = None):
        self.config = config or BinDetectorConfig()

        self._flame_template = None
        self._drop_template = None

        if self.config.symbol_method == SymbolDetectionMethod.TEMPLATE:
            self._load_templates()

    def _load_templates(self):
        if self.config.flame_template_path:
            t = cv2.imread(self.config.flame_template_path)
            if t is not None:
                self._flame_template = cv2.cvtColor(t, cv2.COLOR_BGR2GRAY)
            else:
                print(f"WARNING: flame template not found: {self.config.flame_template_path}")

        if self.config.drop_template_path:
            t = cv2.imread(self.config.drop_template_path)
            if t is not None:
                self._drop_template = cv2.cvtColor(t, cv2.COLOR_BGR2GRAY)
            else:
                print(f"WARNING: drop template not found: {self.config.drop_template_path}")

    def detect(self, frame: np.ndarray) -> BinDetection:
        null = BinDetection(
            detected=False,
            offset_x=0.0,
            offset_y=0.0,
            area_fraction=0.0,
            symbol=None,
            symbol_confidence=0.0,
            bbox=(0, 0, 0, 0),
        )

        if frame is None or frame.size == 0:
            return null

        h, w = frame.shape[:2]

        # Krok 1 — znajdź pojemnik przez HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        bin_mask = cv2.inRange(
            hsv,
            np.array(self.config.bin_hsv_lower),
            np.array(self.config.bin_hsv_upper),
        )

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        bin_mask = cv2.morphologyEx(bin_mask, cv2.MORPH_CLOSE, kernel)
        bin_mask = cv2.morphologyEx(bin_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            bin_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if not contours:
            return null

        # Największy kontur = pojemnik
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        area_fraction = area / (w * h)

        if area_fraction < self.config.min_bin_area_fraction:
            return null

        bx, by, bw, bh = cv2.boundingRect(largest)

        # offset_x, offset_y względem centrum kadru
        cx = bx + bw / 2.0
        cy = by + bh / 2.0
        offset_x = (cx - w / 2.0) / (w / 2.0)
        offset_y = (cy - h / 2.0) / (h / 2.0)

        # Krok 2 — wykryj symbol wewnątrz bounding box pojemnika
        bin_roi = frame[by:by+bh, bx:bx+bw]

        if self.config.symbol_method == SymbolDetectionMethod.HSV:
            symbol, confidence = self._detect_symbol_hsv(bin_roi)
        else:
            symbol, confidence = self._detect_symbol_template(bin_roi)

        return BinDetection(
            detected=True,
            offset_x=float(offset_x),
            offset_y=float(offset_y),
            area_fraction=float(area_fraction),
            symbol=symbol,
            symbol_confidence=float(confidence),
            bbox=(bx, by, bw, bh),
        )

    def _detect_symbol_hsv(
        self, roi: np.ndarray
    ) -> Tuple[Optional[str], float]:
        """
        Wykrywa symbol przez porównanie liczby pikseli pasujących
        do HSV flame vs HSV drop w obszarze pojemnika.
        Im więcej pikseli danego koloru tym wyższy confidence.
        """
        if roi.size == 0:
            return None, 0.0

        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        total_pixels = roi.shape[0] * roi.shape[1]

        flame_mask = cv2.inRange(
            hsv_roi,
            np.array(self.config.flame_hsv_lower),
            np.array(self.config.flame_hsv_upper),
        )
        drop_mask = cv2.inRange(
            hsv_roi,
            np.array(self.config.drop_hsv_lower),
            np.array(self.config.drop_hsv_upper),
        )

        flame_pixels = cv2.countNonZero(flame_mask)
        drop_pixels = cv2.countNonZero(drop_mask)

        if flame_pixels == 0 and drop_pixels == 0:
            return None, 0.0

        if flame_pixels > drop_pixels:
            confidence = flame_pixels / total_pixels
            return "flame", min(confidence * 10, 1.0)
        else:
            confidence = drop_pixels / total_pixels
            return "drop", min(confidence * 10, 1.0)

    def _detect_symbol_template(
        self, roi: np.ndarray
    ) -> Tuple[Optional[str], float]:
        """
        Wykrywa symbol przez template matching.
        Skaluje template do rozmiaru ROI i porównuje przez cv2.matchTemplate.
        Zwraca symbol z wyższym score jeśli przekracza próg.
        """
        if roi.size == 0:
            return None, 0.0

        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        rh, rw = gray_roi.shape

        best_symbol = None
        best_score = 0.0

        for symbol_name, template in [
            ("flame", self._flame_template),
            ("drop", self._drop_template),
        ]:
            if template is None:
                continue

            # Skaluj template do rozmiaru ROI
            scaled = cv2.resize(template, (rw, rh))

            result = cv2.matchTemplate(gray_roi, scaled, cv2.TM_CCOEFF_NORMED)
            _, score, _, _ = cv2.minMaxLoc(result)

            if score > best_score:
                best_score = score
                best_symbol = symbol_name

        if best_score < self.config.template_match_threshold:
            return None, best_score

        return best_symbol, best_score

    def detect_both_methods(
        self, frame: np.ndarray
    ) -> Tuple[BinDetection, BinDetection]:
        """
        Uruchamia obie metody i zwraca wyniki osobno.
        Przydatne w skrypcie kalibracyjnym.
        """
        original_method = self.config.symbol_method

        self.config.symbol_method = SymbolDetectionMethod.HSV
        result_hsv = self.detect(frame)

        self.config.symbol_method = SymbolDetectionMethod.TEMPLATE
        result_template = self.detect(frame)

        self.config.symbol_method = original_method

        return result_hsv, result_template