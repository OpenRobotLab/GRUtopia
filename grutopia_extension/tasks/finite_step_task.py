from omni.isaac.core.scenes import Scene

from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task import BaseTask


@BaseTask.register('FiniteStepTask')
class FiniteStepTask(BaseTask):
    def __init__(self, runtime: TaskRuntime, scene: Scene):
        super().__init__(runtime, scene)
        self.stop_count = 0
        self.max_step = 500
        # Validate task setting
        if not runtime.task_settings:
            pass
        elif not isinstance(runtime.task_settings, dict):
            raise ValueError('task_settings of FiniteStepTask must be a dict')
        if 'max_step' in runtime.task_settings:
            self.max_step = runtime.task_settings['max_step']

    def is_done(self) -> bool:
        self.stop_count += 1
        return self.stop_count > self.max_step
