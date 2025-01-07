import time

from grutopia.core.config.metric import MetricUserConfig
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task.metric import BaseMetric
from grutopia.core.util import log


@BaseMetric.register('DebugMetric')
class DebugMetric(BaseMetric):
    """
    Calculate some info for debug
    """

    def __init__(self, config: MetricUserConfig, task_runtime: TaskRuntime):
        super().__init__(config, task_runtime)
        self.reset()

    def reset(self):
        self.start_time = None
        self.landmarks = None
        self.path = []
        self.question = None
        self.dialogues = []
        self.dialogue_landmarks = {}
        self.dialogue_turn = 0

    def update(self, task_obs: dict):
        """
        This function is called at each world step.
        """
        if self.start_time is None:
            self.start_time = time.time()
        robot_obs = task_obs[self.task_runtime.robots[0].name]
        # log.debug(f'====== obs: {robot_obs} =====')
        if 'render' in robot_obs and robot_obs['render']:
            self.path.append(robot_obs['position'].tolist())
        self.landmarks = robot_obs['camera']['landmarks'] if robot_obs['camera'][
            'landmarks'] is not None else self.landmarks
        if task_obs[self.task_runtime.robots[0].name].get('question') is not None:
            self.question = task_obs[self.task_runtime.robots[0].name].get('question')
        if task_obs[self.task_runtime.robots[0].name].get('answer') is not None:
            self.dialogues.append({
                'question': self.question,
                'answer': task_obs[self.task_runtime.robots[0].name].get('answer')
            })

    def calc(self, task_info: dict):
        """
        This function is called to calculate the metrics when the episode is terminated.
        """
        log.info('DebugMetric calc() called.')
        last_landmarks = [
            landmark.split(',')[1].lower() for landmark in self.landmarks if len(landmark.split(',')) == 2
        ]
        self.dialogue_turn = len(self.dialogues)
        self.end_time = time.time()
        debug_metric = {
            'target': self.task_runtime.extra['target'],
            'question': self.task_runtime.extra['question'],
            'time': self.end_time - self.start_time,
            'last_view': last_landmarks,
            'path': self.path,
            'dialogue_turn': self.dialogue_turn,
            'dialogue': self.dialogues,
            'dialogue_landmarks': self.dialogue_landmarks
        }
        return debug_metric
