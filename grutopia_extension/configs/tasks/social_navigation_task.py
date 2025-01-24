from typing import List, Optional

from pydantic import BaseModel

from grutopia.core.config import EpisodeCfg
from grutopia.core.config.task import TaskCfg


class TaskSettingCfg(BaseModel):
    max_step: int


class SocialNavigationExtra(BaseModel):
    question: Optional[str] = None
    target: str
    distance: float
    start_point: list
    target_point: list


class SocialNavigationEpisodeCfg(EpisodeCfg):
    extra: SocialNavigationExtra


class SocialNavigationTaskCfg(TaskCfg):
    type: Optional[str] = 'SocialNavigationTask'
    task_settings: TaskSettingCfg
    episodes: List[SocialNavigationEpisodeCfg]
