from omni.isaac.core.scenes import Scene

# from grutopia.core.datahub import DataHub
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task import BaseTask


@BaseTask.register('SimpleAgentTask')
class SimpleAgentTask(BaseTask):

    def __init__(self, runtime: TaskRuntime, scene: Scene):
        super().__init__(runtime, scene)
        self.stop_count = 0

    def calculate_metrics(self) -> dict:
        pass

    def is_done(self) -> bool:
        # return DataHub.get_episode_finished(self.runtime.name)
        self.stop_count += 1
        return self.stop_count > 200

    def individual_reset(self):
        for name, metric in self.metrics.items():
            metric.reset()
