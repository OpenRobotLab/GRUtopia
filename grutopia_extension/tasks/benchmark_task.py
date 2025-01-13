import time

from omni.isaac.core.scenes import Scene
from pydantic import BaseModel

from grutopia.core.datahub import DataHub
from grutopia.core.runtime import TaskRuntime
from grutopia.core.task import BaseTask


class BenchmarkTaskModel(BaseModel):
    timeout: float | str = 60.0  # timeout: minute


class BenchmarkTask(BaseTask):
    def __init__(self, runtime: TaskRuntime, scene: Scene):
        super().__init__(runtime, scene)
        config = BenchmarkTaskModel(**runtime.meta)
        self.time_start = round(time.time())
        self.timeout_time = config.timeout * 60 + self.time_start

    def is_done(self) -> bool:
        return DataHub.get_episode_finished(self.runtime.name) or round(time.time()) > self.timeout_time

    def individual_reset(self):
        for name, metric in self.metrics.items():
            metric.reset()
