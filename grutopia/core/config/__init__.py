from typing import List, Optional

from pydantic import BaseModel

from grutopia.core.config.agent import AgentConfig
from grutopia.core.config.robot import RobotModel
from grutopia.core.config.scene import Object, Scene
from grutopia.core.config.task import TaskConfig
from grutopia.core.config.task.episode import EpisodeConfig, EpisodeConfigFile


class SimConfig(BaseModel):
    """
    Config of isaac simulator
    """

    physics_dt: Optional[float | str] = 1 / 60
    rendering_dt: Optional[float | str] = 1 / 60
    rendering_interval: Optional[int] = None
    use_fabric: Optional[bool] = False


class DistributionConfig(BaseModel):
    """
    Config of distribution, only for distributed operation mode
    """

    worker_num: Optional[int] = 1


class Config(BaseModel):
    """
    Config validator for input file (yaml -> dict).
    """

    simulator: Optional[SimConfig] = SimConfig()
    task_config: TaskConfig
    agents: Optional[List[AgentConfig]] = []
