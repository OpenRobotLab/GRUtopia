from typing import Optional

from grutopia.core.config.task import TaskCfg


class FiniteStepTaskCfg(TaskCfg):
    type: Optional[str] = 'FiniteStepTask'
    max_steps: Optional[int] = 500
