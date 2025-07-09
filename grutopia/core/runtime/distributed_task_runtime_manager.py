from typing import Optional, Union

from grutopia.core.config import Config
from grutopia.core.runtime.task_runtime import BaseTaskRuntimeManager, Env, TaskRuntime


# TODO: use ray to implement task runtime manager for the distributed version
class DistributedTaskRuntimeManager(BaseTaskRuntimeManager):
    def __init__(self, config: Config = None):
        pass

    def get_next_task_runtime(self, last_env: Optional[Env] = None) -> Union[TaskRuntime, None]:
        pass
