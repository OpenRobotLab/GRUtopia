from typing import List, Optional

from pydantic import BaseModel

from grutopia.core.config import EpisodeCfg
from grutopia.core.config.task import TaskCfg


class TaskSettingCfg(BaseModel):
    max_step: int


class ManipulationExtra(BaseModel):
    prompt: Optional[str] = ''
    target: str
    episode_idx: int


class ManipulationEpisodeCfg(EpisodeCfg):
    extra: ManipulationExtra


class ManipulationTaskCfg(TaskCfg):
    type: Optional[str] = 'ManipulationTask'
    task_settings: TaskSettingCfg
    episodes: List[ManipulationEpisodeCfg]
