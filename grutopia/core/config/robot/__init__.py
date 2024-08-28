from typing import List, Optional

from pydantic import BaseModel

from grutopia.core.config.robot.params import ControllerParams, SensorParams


class RobotUserConfig(BaseModel):
    # meta info
    name: str
    type: str
    prim_path: str
    create_robot: bool = True

    # common config
    position: Optional[List[float]] = [.0, .0, .0]
    orientation: Optional[List[float]] = None
    scale: Optional[List[float]] = None

    # Parameters
    controller_params: Optional[List[ControllerParams]] = None
    sensor_params: Optional[List[SensorParams]] = None
