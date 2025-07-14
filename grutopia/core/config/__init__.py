from enum import Enum
from typing import Optional

from pydantic import BaseModel

from grutopia.core.config.distribution import DistributionCfg
from grutopia.core.config.object import ObjectCfg
from grutopia.core.config.robot import RobotCfg
from grutopia.core.config.task import TaskCfg
from grutopia.core.config.task.episode import EpisodeCfg, EpisodeConfigFile


class Simulator(Enum):
    ISAACSIM = 'isaac_sim'


class SimConfig(BaseModel):
    """
    Config of isaac simulator
    """

    physics_dt: Optional[float | str] = 1 / 60
    rendering_dt: Optional[float | str] = 1 / 60
    rendering_interval: Optional[int] = None
    use_fabric: Optional[bool] = False
    headless: Optional[bool] = True
    webrtc: Optional[bool] = False
    native: Optional[bool] = False


class Config(BaseModel):
    """
    Config validator for input file (yaml -> dict).
    """

    simulator: Optional[SimConfig] = SimConfig()
    task_config: TaskCfg

    def distribute(self, distribution_config: DistributionCfg):
        distributed_config = DistributedConfig(
            simulator=self.simulator.copy(),
            task_config=self.task_config.copy(),
            distribution_config=distribution_config.copy(),
        )
        return distributed_config


class DistributedConfig(Config):
    distribution_config: Optional[DistributionCfg] = None
