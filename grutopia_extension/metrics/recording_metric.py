from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task.metric import BaseMetric
from grutopia_extension.configs.metrics.recording_metric import RecordingMetricCfg


@BaseMetric.register('RecordingMetric')
class RecordingMetric(BaseMetric):
    """
    Record any controller's output or joint actions during playing or teleoperating
    """

    def __init__(self, config: RecordingMetricCfg, task_runtime: TaskRuntime):
        super().__init__(config, task_runtime)
        self.param = config
        _robot_name = self.param.robot_name
        self.robot_name = (
            _robot_name + '_' + str(self.task_runtime.env.env_id)
        )  # real robot name in isaac sim: {robot_name}_{env_id}
        self.fields = self.param.fields
        self.step = 0
        self.obs = {}

    def reset(self):
        self.step = 0

    def update(self, task_obs: dict):
        """
        This function is called at each world step.
        """
        if self.fields is None:
            self.obs[self.step] = task_obs
        else:
            assert len(task_obs) == 1, f'only support one task currently. but len(task_obs) is : {len(task_obs)}'
            robot_name = list(task_obs.keys())[0]
            self.obs[self.step] = {robot_name: {}}
            for record_field in self.fields:
                self.obs[self.step][robot_name][record_field] = task_obs[robot_name][record_field]

        self.step += 1

    def calc(self):
        """
        This function is called to calculate the metrics when the episode is terminated.
        """
        return self.obs
