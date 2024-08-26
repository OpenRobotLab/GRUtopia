from typing import Any, Dict, List, Optional

from pydantic import BaseModel

from grutopia.core.config.robot import RobotUserConfig
from grutopia.core.config.scene import Object


class EpisodeConfig(BaseModel, extra='allow'):
    """
    Episode config class.
    """
    scene_asset_path: Optional[str] = None
    scene_scale: Optional[List[float]] = [1.0, 1.0, 1.0]
    scene_position: Optional[List[float]] = [0, 0, 0]
    scene_orientation: Optional[List[float]] = [1.0, 0, 0, 0]
    robots: Optional[List[RobotUserConfig]] = []
    objects: Optional[List[Object]] = []
    meta: Optional[Dict[str, Any]] = {}


class EpisodeConfigFile(BaseModel, extra='allow'):
    """
    Episode config file model.
    """
    episodes: List[EpisodeConfig]
