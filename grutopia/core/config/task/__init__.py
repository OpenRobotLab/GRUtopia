from typing import List, Optional

import numpy as np
from pydantic import BaseModel, Extra

from grutopia.core.config.metric import MetricUserConfig
from grutopia.core.config.robot import RobotUserConfig
from grutopia.core.config.scene import Object


class TaskUserConfig(BaseModel, extra=Extra.allow):
    type: str
    name: str

    # scene
    scene_asset_path: Optional[str] = None
    scene_scale: Optional[List[float]] = [1.0, 1.0, 1.0]
    scene_position: Optional[List[float]] = [0, 0, 0]
    scene_orientation: Optional[List[float]] = [1.0, 0, 0, 0]

    # inherit
    robots: Optional[List[RobotUserConfig]] = []
    objects: Optional[List[Object]] = []
    metrics: Optional[List[MetricUserConfig]] = []

    # path
    root_path: str
    scene_root_path: str = '/scene'
    robots_root_path: str = '/robots'
    objects_root_path: str = '/objects'

    # offset
    offset: Optional[List[float]] = None
    offset_size: float = 10.0

    # id
    env_id: int = 0
