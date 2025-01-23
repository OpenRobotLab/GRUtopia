from typing import List, Optional

from pydantic import BaseModel

from grutopia.core.config import EpisodeConfig
from grutopia.core.config.task import TaskConfig


class TaskSettingCfg(BaseModel):
    max_step: int


class SocialNavigationExtra(BaseModel):
    question: Optional[str] = None
    target: str
    distance: float
    start_point: list
    target_point: list


class SocialNavigationEpisodeCfg(EpisodeConfig):
    extra: SocialNavigationExtra


class SocialNavigationTaskCfg(TaskConfig):
    type: Optional[str] = 'SocialNavigationTask'
    task_settings: TaskSettingCfg
    episodes: List[SocialNavigationEpisodeCfg]
