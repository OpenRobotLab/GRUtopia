from typing import Optional, Tuple

from grutopia.core.config.robot import SensorModel


class CameraModel(SensorModel):
    # Fields from params.
    enable: Optional[bool] = True
    size: Optional[Tuple[int, int]] = None


class RepCameraModel(SensorModel):
    # Fields from params.
    enable: Optional[bool] = True
    size: Optional[Tuple[int, int]] = None  # Camera only
