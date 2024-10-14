# import time
from typing import Optional

from omni.isaac.core.scenes import Scene
from pydantic import BaseModel

from grutopia.core.datahub import DataHub
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task import BaseTask
from grutopia.core.util import log


class TaskSettingModel(BaseModel):
    max_step: int
    # verbose: str  # Agent
    # fall_threshold: str  # Agent
    # max_dialogue: str  # NPC


class NavigationExtraTaskInfoModel(BaseModel):
    prompt: Optional[str] = ''


@BaseTask.register('SocialBenchmarkTask')
class SocialBenchmarkTask(BaseTask):

    def __init__(self, runtime: TaskRuntime, scene: Scene):
        super().__init__(runtime, scene)
        self.step_counter = 0
        self.settings = TaskSettingModel(**runtime.task_settings)
        self.extra = NavigationExtraTaskInfoModel(**runtime.extra)
        log.info(f'task_settings: max_step       : {self.settings.max_step}.)')
        # Episode
        log.info(f'Episode meta : prompt         : {self.extra.prompt}.)')

    def is_done(self) -> bool:
        self.step_counter = self.step_counter + 1
        return DataHub.get_episode_finished(self.runtime.name) or self.step_counter > self.settings.max_step

    def individual_reset(self):
        for name, metric in self.metrics.items():
            metric.reset()
