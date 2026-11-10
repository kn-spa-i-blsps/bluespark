from enum import IntEnum 
from dataclasses import dataclass


class PayloadAction(IntEnum):
    """Enum for payload actions."""
    DROP_BALL_1 = 0
    DROP_BALL_2= 1
    FIRE_TORPEDO = 2
    RESET = 99


@dataclass
class DetectionResult:
    class_id: int
    class_name: str
    confidence: float
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    source_node: str
    stamp_ns: int
