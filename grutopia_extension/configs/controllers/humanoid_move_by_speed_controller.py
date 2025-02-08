from typing import List, Optional

from grutopia.core.config.robot import ControllerCfg


class HumanoidMoveBySpeedControllerCfg(ControllerCfg):

    type: Optional[str] = 'HumanoidMoveBySpeedController'
    joint_names: List[str]
    policy_weights_path: str
