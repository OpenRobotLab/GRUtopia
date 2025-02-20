from grutopia.core.config.metric import MetricCfg
from grutopia.core.datahub.datahub import DataHub
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task.metric import BaseMetric
from grutopia.core.util import log


@BaseMetric.register('ResetTimeMetric')
class ResetTimeMetric(BaseMetric):
    """
    Calculate the fall times of the robot
    """

    def __init__(self, config: MetricCfg, task_runtime: TaskRuntime):
        super().__init__(config, task_runtime)
        self.reset()

    def reset(self):
        self.fall_times = 0

    def update(self, task_obs: dict):
        """
        This function is called at each world step.
        """
        action = DataHub.get_actions_by_task_name(self.task_runtime.name)
        # log.debug(f"======== get action: {action} ========")
        self.fall_times = (
            self.fall_times + 1
            if action and len(action['controllers']) > 0 and 'recover' == action['controllers'][0]['controller']
            else self.fall_times
        )

    def calc(self, task_info: dict):
        """
        This function is called to calculate the metrics when the episode is terminated.
        """
        log.info('ResetTimeMetric calc() called.')

        return self.fall_times
