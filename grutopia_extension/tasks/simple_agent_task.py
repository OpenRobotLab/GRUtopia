from omni.isaac.core.scenes import Scene

from grutopia.core.datahub import DataHub
from grutopia.core.runtime import TaskRuntime
from grutopia.core.task import BaseTask


@BaseTask.register('SimpleAgentTask')
class SimpleAgentTask(BaseTask):

    def __init__(self, runtime: TaskRuntime, scene: Scene):
        super().__init__(runtime, scene)

    def calculate_metrics(self) -> dict:
        pass

    def is_done(self) -> bool:
        return DataHub.get_episode_finished(self.runtime.name)

    def individual_reset(self):
        for name, metric in self.metrics.items():
            metric.reset()
