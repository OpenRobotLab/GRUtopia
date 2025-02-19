from typing import Any, Dict, List, Optional

from pydantic import BaseModel

from grutopia.core.config import EpisodeCfg
from grutopia.core.config.task import TaskCfg


class TaskSettingCfg(BaseModel):
    max_step: int
    verbose: Optional[bool] = False


class MobileManipulationEpisodeCfg(EpisodeCfg):
    extra: Optional[Dict[str, Any]] = {}


class MobileManipulationTaskCfg(TaskCfg):
    type: Optional[str] = 'MobileManipulationTask'
    task_settings: TaskSettingCfg
    episodes: List[MobileManipulationEpisodeCfg]
