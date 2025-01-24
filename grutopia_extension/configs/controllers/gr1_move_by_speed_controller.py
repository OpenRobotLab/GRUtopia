from typing import List, Optional

from grutopia.core.config.robot import ControllerModel


class GR1MoveBySpeedControllerCfg(ControllerModel):
    type: Optional[str] = 'GR1MoveBySpeedController'
    joint_names: List[str]
    policy_weights_path: str
