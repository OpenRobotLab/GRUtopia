from typing import List, Optional

from grutopia.core.config.robot import ControllerModel


class AliengoMoveBySpeedControllerCfg(ControllerModel):

    type: Optional[str] = 'AliengoMoveBySpeedController'
    joint_names: List[str]
    policy_weights_path: str
