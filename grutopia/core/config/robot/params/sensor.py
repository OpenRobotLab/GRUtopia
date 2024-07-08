# SensorParams
from typing import Optional, Tuple

from pydantic import BaseModel


class SensorParams(BaseModel):
    """
    Sensor config validator
    """
    name: str
    enable: Optional[bool] = True
    size: Optional[Tuple[int, int]]  # Camera only
    scan_rate: Optional[int]  # RPS. Lidar only
