from typing import List, Optional

from grutopia.core.config.robot import ControllerCfg


class GR1TeleOpControllerCfg(ControllerCfg):

    type: Optional[str] = 'GR1TeleOpController'
    joint_names: List[str]
