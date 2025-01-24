from typing import List, Optional

from grutopia.core.config.robot import ControllerModel


class HumanoidMoveBySpeedControllerCfg(ControllerModel):

    type: Optional[str] = 'HumanoidMoveBySpeedController'
    joint_names: List[str]
    policy_weights_path: str
