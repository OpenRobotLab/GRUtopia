import numpy as np
from pydantic import BaseModel

from internutopia.core.config import TaskCfg
from internutopia.core.config.metric import MetricCfg
from internutopia.core.task.metric import BaseMetric
from internutopia.core.util import log


class SimpleMetricParam(BaseModel):
    robot_name: str


@BaseMetric.register('SimpleMetric')
class SimpleMetric(BaseMetric):
    """
    Calculate the total distance a robot moves
    """

    def __init__(self, config: MetricCfg, task_config: TaskCfg):
        super().__init__(config, task_config)
        self.distance: float = 0.0
        self.position = None
        self.param = SimpleMetricParam(**config.metric_config)
        self.robot_name = self.param.robot_name

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
