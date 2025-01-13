import numpy as np
from pydantic import BaseModel

from grutopia.core.config.metric import MetricUserConfig
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task.metric import BaseMetric
from grutopia.core.util import log


class SimpleMetricParam(BaseModel):
    robot_name: str


@BaseMetric.register('SimpleMetric')
class SimpleMetric(BaseMetric):
    """
    Calculate the total distance a robot moves
    """

    def __init__(self, config: MetricUserConfig, task_runtime: TaskRuntime):
        super().__init__(config, task_runtime)
        self.distance: float = 0.0
        self.position = None
        self.param = SimpleMetricParam(**config.metric_config)
        _robot_name = self.param.robot_name
        self.robot_name = (
            _robot_name + '_' + str(self.task_runtime.env.env_id)
        )  # real robot name in isaac sim: {robot_name}_{env_id}

    def reset(self):
        self.distance = 0.0

    def update(self, task_obs: dict):
        """
        This function is called at each world step.
        """
        if self.position is None:
            self.position = task_obs[self.robot_name]['position']
            return
        self.distance += np.linalg.norm(self.position - task_obs[self.robot_name]['position'])
        self.position = task_obs[self.robot_name]['position']
        # log.info(f'distance: {self.distance}')
        return

    def calc(self):
        """
        This function is called to calculate the metrics when the episode is terminated.
        """
        log.info('SimpleMetric calc() called.')
        return self.distance
