from typing import Optional

from grutopia.core.config.task import TaskCfg


class SingleInferenceTaskCfg(TaskCfg):
    type: Optional[str] = 'SingleInferenceTask'
