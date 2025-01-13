import numpy as np
from pydantic import BaseModel

from grutopia.core.config.metric import MetricUserConfig
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task.metric import BaseMetric
from grutopia.core.util import log


class SuccessParam(BaseModel):
    navigation_error_threshold: float


@BaseMetric.register('SocialNavigationSuccessMetric')
class SocialNavigationSuccessMetric(BaseMetric):
    """
    Calculate the success of this episode
    """

    def __init__(self, config: MetricUserConfig, task_runtime: TaskRuntime):
        super().__init__(config, task_runtime)
        self.param = SuccessParam(**config.metric_config)
        self.navigation_error_threshold = self.param.navigation_error_threshold

        self.reset()

    def reset(self):
        self.landmarks = None
        self.position = None
        self.distance = 0.0

    def update(self, task_obs: dict):
        """
        This function is called at each world step.
        """
        robot_obs = task_obs[self.task_runtime.robots[0].name]
        if self.position is None:
            self.position = robot_obs['position'][:2]
            return
        self.distance += np.linalg.norm(self.position[:2] - robot_obs['position'][:2])
        self.position = robot_obs['position'][:2]
        self.landmarks = (
            robot_obs['camera']['landmarks'] if robot_obs['camera']['landmarks'] is not None else self.landmarks
        )

    def calc(self, task_info: dict):
        """
        This function is called to calculate the metrics when the episode is terminated.
        """
        log.info('SocialNavigationSuccessMetric calc() called.')

        last_landmarks = [
            landmark.split(',')[1].lower() for landmark in self.landmarks if len(landmark.split(',')) == 2
        ]
        success_view = task_info['target'].lower() in last_landmarks
        navigation_error = np.linalg.norm(np.array(self.position) - task_info['target_point'][:2])

        success = success_view and navigation_error <= self.navigation_error_threshold
        spl = success * task_info['distance'] / max(self.distance, task_info['distance'])
        # make sure all the result is in Python type
        return {'pl': float(self.distance), 'success': bool(success), 'spl': float(spl)}
