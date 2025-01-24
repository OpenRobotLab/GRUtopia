from typing import List, Optional

from pydantic import BaseModel

from grutopia.core.config import EpisodeCfg
from grutopia.core.config.task import TaskCfg


class TaskSettingCfg(BaseModel):
    max_step: int


class MobileManipulationExtra(BaseModel):
    instruction: str
    target: str
    meta_path: str
    start_point: list
    conditions: List[dict]


class MobileManipulationEpisodeCfg(EpisodeCfg):
    extra: MobileManipulationExtra


class MobileManipulationTaskCfg(TaskCfg):
    type: Optional[str] = 'MobileManipulationTask'
    task_settings: TaskSettingCfg
    episodes: List[MobileManipulationEpisodeCfg]
