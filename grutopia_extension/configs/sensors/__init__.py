from typing import Optional, Tuple

from grutopia.core.config.robot import SensorModel


class CameraCfg(SensorModel):
    # Fields from params.
    type: Optional[str] = 'Camera'
    enable: Optional[bool] = True
    size: Optional[Tuple[int, int]] = None


class RepCameraCfg(SensorModel):
    # Fields from params.
    type: Optional[str] = 'RepCamera'
    enable: Optional[bool] = True
    size: Optional[Tuple[int, int]] = None  # Camera only


class MocapControlledCameraCfg(SensorModel):
    # Fields from params.
    type: Optional[str] = 'MocapControlledCamera'
    enable: Optional[bool] = True
    size: Optional[Tuple[int, int]] = None  # Camera only
    translation: Optional[Tuple[float, float, float]] = None
    orientation: Optional[Tuple[float, float, float, float]] = None  # Quaternion in local frame
