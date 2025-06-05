from omni.isaac.core.scenes import Scene

from grutopia.core.datahub import DataHub
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task import BaseTask
from grutopia.core.util import log
from grutopia_extension.configs.tasks.manipulation_task import (
    ManipulationExtra,
    TaskSettingCfg,
)


@BaseTask.register('ManipulationTask')
class ManipulationTask(BaseTask):
    def __init__(self, runtime: TaskRuntime, scene: Scene):
        super().__init__(runtime, scene)
        self.step_counter = 0
        if isinstance(runtime.task_settings, TaskSettingCfg):
            self.settings = runtime.task_settings
        else:
            raise ValueError('task_settings must be a TaskSettingCfg')
        if isinstance(runtime.extra, ManipulationExtra):
            self.extra = runtime.extra
        else:
            raise ValueError('extra must be a ManipulationExtra')
        log.info(f'task_settings: max_step       : {self.settings.max_step}.)')
        # Episode
        log.info(f'Episode meta : prompt         : {self.extra.prompt}.)')

    def is_done(self) -> bool:
        self.step_counter = self.step_counter + 1
        return DataHub.get_episode_finished(self.runtime.name) or self.step_counter > self.settings.max_step

    def update_metrics(self):
        return super().update_metrics()

    def calculate_metrics(self) -> dict:
        metrics_res = {}
        for name, metric in self.metrics.items():
            metrics_res[name] = metric.calc()

        return metrics_res
