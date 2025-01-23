from typing import Any, List, Optional

from grutopia.core.config import EpisodeConfig
from grutopia.core.config.task import TaskConfig


class SimpleAgentEpisodeCfg(EpisodeConfig):
    extra: Optional[Any] = None


class SimpleAgentTaskCfg(TaskConfig):
    type: Optional[str] = 'SimpleAgentTask'
    episodes: List[SimpleAgentEpisodeCfg]
