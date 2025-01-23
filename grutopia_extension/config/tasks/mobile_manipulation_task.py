from typing import List, Optional

from pydantic import BaseModel

from grutopia.core.config import EpisodeConfig
from grutopia.core.config.task import TaskConfig


class TaskSettingCfg(BaseModel):
    max_step: int


class MobileManipulationExtra(BaseModel):
    instruction: str
    target: str
    meta_path: str
    start_point: list
    conditions: List[dict]


class MobileManipulationEpisodeCfg(EpisodeConfig):
    extra: MobileManipulationExtra


class MobileManipulationTaskCfg(TaskConfig):
    type: Optional[str] = 'MobileManipulationTask'
    task_settings: TaskSettingCfg
    episodes: List[MobileManipulationEpisodeCfg]
