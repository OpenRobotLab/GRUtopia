from typing import Any, List, Optional

from grutopia.core.config import EpisodeCfg
from grutopia.core.config.task import TaskCfg


class SimpleAgentEpisodeCfg(EpisodeCfg):
    extra: Optional[Any] = None


class SimpleAgentTaskCfg(TaskCfg):
    type: Optional[str] = 'SimpleAgentTask'
    episodes: List[SimpleAgentEpisodeCfg]
