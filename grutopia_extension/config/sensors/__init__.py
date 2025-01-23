from typing import Optional, Tuple

from grutopia.core.config.robot import SensorModel


class CameraCfg(SensorModel):
    # Fields from params.
    enable: Optional[bool] = True
    size: Optional[Tuple[int, int]] = None


class RepCameraCfg(SensorModel):
    # Fields from params.
    enable: Optional[bool] = True
    size: Optional[Tuple[int, int]] = None  # Camera only
