from typing import Any, List, Optional

from grutopia.core.config import EpisodeConfig
from grutopia.core.config.task import TaskConfig


class SingleInferenceEpisodeCfg(EpisodeConfig):
    extra: Optional[Any] = None


class SingleInferenceTaskCfg(TaskConfig):
    type: Optional[str] = 'SingleInferenceTask'
    episodes: List[SingleInferenceEpisodeCfg]
