from typing import List, Optional

from grutopia.core.config.robot import ControllerModel


class JointControllerCfg(ControllerModel):

    type: Optional[str] = 'JointController'
    joint_names: List[str]
