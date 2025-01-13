# import time
import traceback
from typing import Any, Dict, Optional

from omni.isaac.core.scenes import Scene
from pydantic import BaseModel

from grutopia.core.datahub import DataHub
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task import BaseTask
from grutopia.core.util import log


class TaskSettingModel(BaseModel):
    max_step: int


class NavigationMetaModel(BaseModel):
    question: Optional[str] = None
    target: str
    distance: float
    start_point: list
    target_point: list


@BaseTask.register('SocialBenchmarkTask')
class SocialBenchmarkTask(BaseTask):
    def __init__(self, runtime: TaskRuntime, scene: Scene):
        super().__init__(runtime, scene)
        self.step_counter = 0
        self.settings = TaskSettingModel(**runtime.task_settings)
        self.episode_meta = NavigationMetaModel(**runtime.extra)
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
