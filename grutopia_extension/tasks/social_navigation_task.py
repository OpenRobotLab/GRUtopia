# import time
import traceback
from typing import Any, Dict

from omni.isaac.core.scenes import Scene

from grutopia.core.datahub import DataHub
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task import BaseTask
from grutopia.core.util import log
from grutopia_extension.configs.tasks.social_navigation_task import (
    SocialNavigationExtra,
    TaskSettingCfg,
)


@BaseTask.register('SocialNavigationTask')
class SocialNavigationTask(BaseTask):
    def __init__(self, runtime: TaskRuntime, scene: Scene):
        super().__init__(runtime, scene)
        self.step_counter = 0
        if isinstance(runtime.task_settings, TaskSettingCfg):
            self.settings = runtime.task_settings
        else:
            raise ValueError('task_settings must be a TaskSettingCfg')
        if isinstance(runtime.extra, SocialNavigationExtra):
            self.episode_meta = runtime.extra
        else:
            raise ValueError('extra must be a SocialNavigationExtra')
        log.info(f'task_settings: max_step       : {self.settings.max_step}.)')
        # Episode
        log.info(f'Episode meta : question         : {self.episode_meta.question}.)')

    def get_observations(self) -> Dict[str, Any]:
        """
        Returns current observations from the objects needed for the behavioral layer.

        Return:
            Dict[str, Any]: observation of robots in this task
        """
        if not self.work:
            return {}
        obs = {}
        for robot_name, robot in self.robots.items():
            try:
                obs[robot_name] = robot.get_obs()
                obs[robot_name]['sim_step'] = self.steps
            except Exception as e:
                log.error(self.name)
                log.error(e)
                traceback.print_exc()
                return {}
        return obs

    def calculate_metrics(self) -> Dict:
        metrics_res = {}
        for name, metric in self.metrics.items():
            metrics_res[name] = metric.calc(dict(self.episode_meta))

        return metrics_res

    def is_done(self) -> bool:
        self.step_counter = self.step_counter + 1
        return DataHub.get_episode_finished(self.runtime.name) or self.step_counter > self.settings.max_step

    def individual_reset(self):
        for name, metric in self.metrics.items():
            metric.reset()
