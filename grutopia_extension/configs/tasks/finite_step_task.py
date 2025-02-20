from typing import List, Optional

from grutopia.core.config import EpisodeCfg
from grutopia.core.config.task import TaskCfg


class FiniteStepTaskEpisodeCfg(EpisodeCfg):
    pass


class FiniteStepTaskCfg(TaskCfg):
    type: Optional[str] = 'FiniteStepTask'
    episodes: List[FiniteStepTaskEpisodeCfg]
