from typing import Optional, Tuple

from grutopia.core.config.robot import ControllerModel


class FrankaMocapTeleopControllerCfg(ControllerModel):
    type: Optional[str] = 'FrankaMocapTeleopController'
    scale: Tuple[float, float, float]
    target_position: Tuple[float, float, float]
    origin_xyz: Optional[Tuple[float, float, float]] = None
    origin_xyz_angle: Optional[Tuple[float, float, float]] = None
