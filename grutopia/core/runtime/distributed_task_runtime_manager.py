from copy import deepcopy
from threading import Lock
from typing import Dict, Optional, Union

import numpy as np

from grutopia.core.config import Config, EpisodeCfg
from grutopia.core.datahub import DataHub
from grutopia.core.runtime.task_runtime import (
    BaseTaskRuntimeManager,
    Env,
    TaskRuntime,
    setup_offset_for_assets,
)


class DistributedTaskRuntimeManager(BaseTaskRuntimeManager):
    def __init__(self, config: Config = None):
        super().__init__(config)
        self.env_num = config.task_config.env_num
        self.proc_num = config.distribution_config.proc_num
        self.lock = Lock()
        self._column_length = int(np.sqrt(self.env_num))
        self._current_env_ids = [0 for _ in range(self.proc_num)]
        self._current_runtimes = [{} for _ in range(self.proc_num)]

    def _pop_next_episode(self) -> Union[EpisodeCfg, None]:
        if self.episodes:
            return self.episodes.pop()
        return None

    def _create_env(self, env_id) -> Env:
        row = int(env_id // self._column_length)
        column = env_id % self._column_length
        offset = [row * self.offset_size, column * self.offset_size, 0]
        new_env = {'env_id': env_id, 'offset': offset}
        return Env(**new_env)

    def get_next_task_runtime(self, last_env: Optional[Env] = None, runner_id=0) -> Union[TaskRuntime, None]:

        with self.lock:
            if last_env is not None:
                env_id = last_env.env_id
            else:
                env_id = self._current_env_ids[runner_id]

            if last_env is None and env_id >= self.env_num:
                raise ValueError('Too many sub envs have been created.')

            next_episode: EpisodeCfg = self._pop_next_episode()
            print(f'next episode is {next_episode}')
            if next_episode is None:
                if last_env is not None:
                    del self._current_runtimes[runner_id][env_id]
                self.all_allocated = True
                return None
            next_episode_template_dict = deepcopy(self.runtime_template)
            if last_env is None:
                last_env = self._create_env(env_id)
                self._current_env_ids[runner_id] = self._current_env_ids[runner_id] + 1
            task_idx = DataHub.gen_task_idx()
            next_episode_template_dict['name'] = f'{self.task_config.type}_{str(task_idx)}'
            next_episode_template_dict['task_idx'] = str(task_idx)
            next_episode_template_dict['root_path'] = f'/World/env_{str(env_id)}'
            next_episode_template_dict['env'] = last_env
            task_runtime = TaskRuntime(**next_episode_template_dict)
            setup_offset_for_assets(
                next_episode,
                last_env,
                next_episode_template_dict['root_path'],
                next_episode_template_dict['robots_root_path'],
                next_episode_template_dict['objects_root_path'],
            )
            for k, v in next_episode.__dict__.items():
                task_runtime.__dict__.update({k: v})

            if env_id in self._current_runtimes[runner_id]:
                del self._current_runtimes[runner_id][env_id]
            self._current_runtimes[runner_id][env_id] = task_runtime

            return task_runtime

    def all_task_allocated(self) -> bool:
        """
        Return if all tasks are allocated
        """
        with self.lock:
            return self.all_allocated

    def active_runtime(self) -> Dict[int, TaskRuntime]:
        """
        Get active runtimes with env id as key.
        """
        active_runtimes = {}
        for runner_id, runtime_map in enumerate(self._current_runtimes):
            for env_id, active_runtime in runtime_map.items():
                index = self.env_num * runner_id + env_id
                active_runtimes[index] = active_runtime

        return active_runtimes
