from typing import Optional

from grutopia.core.config.robot import ControllerCfg


class MoveAlongPathPointsControllerCfg(ControllerCfg):

    type: Optional[str] = 'MoveAlongPathPointsController'
    forward_speed: Optional[float] = None
    rotation_speed: Optional[float] = None
    threshold: Optional[float] = None
