from typing import Any, List, Optional

from grutopia.core.config import EpisodeCfg
from grutopia.core.config.task import TaskCfg


class SingleInferenceEpisodeCfg(EpisodeCfg):
    extra: Optional[Any] = None


class SingleInferenceTaskCfg(TaskCfg):
    type: Optional[str] = 'SingleInferenceTask'
    episodes: List[SingleInferenceEpisodeCfg]
