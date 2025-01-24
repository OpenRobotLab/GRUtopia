from typing import Optional

from grutopia.core.config.robot import ControllerModel


class GripperControllerCfg(ControllerModel):
    type: Optional[str] = 'GripperController'
