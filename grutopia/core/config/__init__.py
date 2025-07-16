from enum import Enum
from typing import List, Optional

from pydantic import BaseModel

from grutopia.core.config.distribution import DistributionCfg
from grutopia.core.config.object import ObjectCfg
from grutopia.core.config.robot import RobotCfg
from grutopia.core.config.task import TaskCfg


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
    env_num: Optional[int] = 1
    env_offset_size: Optional[float] = 5.0
    metrics_save_path: Optional[str] = 'console'
    task_configs: List[TaskCfg]

    def distribute(self, distribution_config: DistributionCfg):
        distributed_config = DistributedConfig(
            simulator=self.simulator,
            env_num=self.env_num,
            env_offset_size=self.env_offset_size,
            metrics_save_path=self.metrics_save_path,
            task_configs=self.task_configs,
            distribution_config=distribution_config,
        )
        return distributed_config


class DistributedConfig(Config):
    distribution_config: Optional[DistributionCfg] = None
