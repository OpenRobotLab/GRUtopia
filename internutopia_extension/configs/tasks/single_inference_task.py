from typing import Optional

from internutopia.core.config.task import TaskCfg


class SingleInferenceTaskCfg(TaskCfg):
    type: Optional[str] = 'SingleInferenceTask'
