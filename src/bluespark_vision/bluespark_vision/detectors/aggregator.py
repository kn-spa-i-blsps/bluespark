from collections import deque, defaultdict
from threading import Lock
from ..utils.types import DetectionResult


class DetectionAggregator:
    def __init__(self, history_size: int = 10):
        self._history_size = history_size
        self._detections: dict[str, deque] = defaultdict(lambda: deque(maxlen=history_size))
        self._lock = Lock()

    def add_detection(self, detection: DetectionResult):
        with self._lock:
            self._detections[detection.source_node].append(detection)

    def get_detections(self, source_node: str) -> list[DetectionResult]:
        with self._lock:
            return list(self._detections[source_node])
        
    def get_all(self) -> dict[str, list[DetectionResult]]:
        with self._lock:
            return {node: list(detections) for node, detections in self._detections.items()}