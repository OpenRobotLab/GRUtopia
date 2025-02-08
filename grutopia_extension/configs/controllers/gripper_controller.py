from typing import Optional

from grutopia.core.config.robot import ControllerCfg


class GripperControllerCfg(ControllerCfg):
    type: Optional[str] = 'GripperController'
