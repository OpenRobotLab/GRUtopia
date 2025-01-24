from typing import Optional

from grutopia.core.config.robot import ControllerModel


class MoveAlongPathPointsControllerCfg(ControllerModel):

    type: Optional[str] = 'MoveAlongPathPointsController'
    forward_speed: Optional[float] = None
    rotation_speed: Optional[float] = None
    threshold: Optional[float] = None
