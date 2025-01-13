from enum import Enum
from typing import List, Optional

from pydantic import BaseModel

from grutopia.core.config.agent import AgentConfig
from grutopia.core.config.robot import RobotUserConfig
from grutopia.core.config.scene import Object, Scene
from grutopia.core.config.task import TaskConfig
from grutopia.core.config.task.episode import EpisodeConfig, EpisodeConfigFile


class DataHubMode(str, Enum):
    local = 'local'
    remote = 'remote'


class DataHubConfig(BaseModel):
    """
    DataHub config
    """

    sim: Optional[DataHubMode] = DataHubMode.local
    chat: Optional[DataHubMode] = DataHubMode.remote
    remote: str = '127.0.0.1:9000'


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


class ValidatedConfig(BaseModel):
    """
    Config validator for input file (yaml -> dict).
    """

    datahub_config: Optional[DataHubConfig] = DataHubConfig()
    simulator: Optional[SimConfig] = SimConfig()
    task_config: TaskConfig
    agents: Optional[List[AgentConfig]] = []
