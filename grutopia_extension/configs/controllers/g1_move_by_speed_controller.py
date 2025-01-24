from typing import List, Optional

from grutopia.core.config.robot import ControllerModel


class G1MoveBySpeedControllerCfg(ControllerModel):
    type: Optional[str] = 'G1MoveBySpeedController'
    joint_names: List[str]
    policy_weights_path: str
