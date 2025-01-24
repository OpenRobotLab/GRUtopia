from typing import Optional

from grutopia.core.config.robot import ControllerModel


class RotateControllerCfg(ControllerModel):
    type: Optional[str] = 'RotateController'
    rotation_speed: Optional[float] = None
    threshold: Optional[float] = None
