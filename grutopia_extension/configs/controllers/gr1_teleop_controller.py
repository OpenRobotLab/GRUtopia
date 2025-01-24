from typing import List, Optional

from grutopia.core.config.robot import ControllerModel


class GR1TeleOpControllerCfg(ControllerModel):

    type: Optional[str] = 'GR1TeleOpController'
    joint_names: List[str]
