from copy import deepcopy
from typing import Optional, Union

import numpy as np

from grutopia.core.config import EpisodeConfig, TaskConfig
from grutopia.core.datahub import DataHub
from grutopia.core.runtime.task_runtime import (
    BaseTaskRuntimeManager,
    Env,
    TaskRuntime,
    setup_offset_for_assets,
)


@BaseTaskRuntimeManager.register('LocalTaskRuntimeManager')
class LocalTaskRuntimeManager(BaseTaskRuntimeManager):
    """
    Task Runtime Manager

    - Init task_runtime_manager at first time (with env_num)
    - Gen runtime for task reload (reuse env_id, offset and so on).

    Attributes:
        env_num (int): Env number.
        task_name_prefix (str): Task name prefix.
        task_config (TaskConfig): Task config that user input.
        episodes (List[EpisodeConfig]): Rest episodes.
        active_runtimes (str): Activated runtimes, format like => { env_id: TaskRuntime } .
        offset_size (float): Offset size.
    """

    def __init__(self, task_user_config: TaskConfig = None):
        """
        Args:
            task_user_config (TaskConfig): Task config read from user input config file.
        """
        super().__init__(task_user_config=task_user_config)
        self._current_env_id = 0
        self._column_length = int(np.sqrt(self.env_num))
        self._loop = False

    def _pop_next_episode(self) -> Union[EpisodeConfig, None]:
        """
        Pop next episode from episodes' list.

        Returns:
            EpisodeConfig: next episode assets. To be assembled to TaskRuntime.
        """
        if self.episodes:
            return self.episodes.pop()
        return None

    def get_next_task_runtime(self, last_env: Optional[Env] = None) -> Union[TaskRuntime, None]:
        """
        Get next task runtime with env of last episode.
        Clean last task runtime in active_runtimes.
        Set new task runtime to active_runtimes.

        Args:
            last_env (Env):  Env of last episode.

        Returns:
            TaskRuntime: TaskRuntime for next task.

        Raises:
            ValueError: If too many sub envs have been created.
        """
        if self._loop:
            print('===================== loop =====================')
            return [i for i in self.active_runtimes.values()][0]

        if last_env is None and self._current_env_id >= self.env_num:
            raise ValueError('Too many sub envs have been created.')

        next_episode = self._pop_next_episode()
        if next_episode is None:
            if last_env is not None:
                del self.active_runtimes[str(last_env.env_id)]
            self.all_allocated = True
            return None

        assets = next_episode.model_dump()
        next_episode_dict = deepcopy(self.runtime_template)
        next_episode_dict.update(**assets)

        # create last_env if not exist
        if last_env is None:
            last_env = self._create_env()

        # Update task_idx
        task_idx = DataHub.gen_task_idx()
        next_episode_dict['name'] = f'{self.task_config.task_name_prefix}_{str(task_idx)}'
        next_episode_dict['task_idx'] = task_idx
        next_episode_dict['root_path'] = f'/World/env_{str(last_env.env_id)}'
        next_episode_dict['env'] = last_env

        setup_offset_for_assets(next_episode_dict)
        task_runtime = TaskRuntime(**next_episode_dict)
        if str(last_env.env_id) in self.active_runtimes:
            del self.active_runtimes[str(last_env.env_id)]
        self.active_runtimes[str(last_env.env_id)] = task_runtime

        # Change _loop param after the first reset
        # TODO: Make it more elegant.
        self._loop = self.loop

        return task_runtime

    def init(self):
        """
        Initialize of task_runtime_manager. (only run once)
        Initialize the first batch of active_runtimes to be simulated simultaneously according to `env_num`.

        Set init Env.
            1. Set up env_id
            2. Set up offset in isaac sim (2D tuple)
        """
        _column_length = int(np.sqrt(self.env_num))
        for env_id in range(self.env_num):
            row = int(env_id // _column_length)
            column = env_id % _column_length
            offset = [row * self.offset_size, column * self.offset_size, 0]
            new_env = {'env_id': env_id, 'offset': offset}
            task_runtime = self.get_next_task_runtime(Env(**new_env))
            self.active_runtimes[str(env_id)] = task_runtime

    def _create_env(self) -> Env:
        row = int(self._current_env_id // self._column_length)
        column = self._current_env_id % self._column_length
        offset = [row * self.offset_size, column * self.offset_size, 0]
        new_env = {'env_id': self._current_env_id, 'offset': offset}
        self._current_env_id += 1
        return Env(**new_env)
