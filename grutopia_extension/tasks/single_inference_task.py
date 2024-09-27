from omni.isaac.core.scenes import Scene

from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task import BaseTask


@BaseTask.register('SingleInferenceTask')
class SimpleInferenceTask(BaseTask):

    def __init__(self, runtime: TaskRuntime, scene: Scene):
        super().__init__(runtime, scene)

    def calculate_metrics(self) -> dict:
        pass

    def is_done(self) -> bool:
        return False

    def individual_reset(self):
        for name, metric in self.metrics.items():
            metric.reset()
