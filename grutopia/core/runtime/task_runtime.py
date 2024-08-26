from copy import deepcopy
from typing import Any, Dict, List, Optional, Union

import numpy as np
from pydantic import BaseModel, Extra

from grutopia.core.config import EpisodeConfig, Object, RobotUserConfig, TaskConfig
from grutopia.core.config.metric import MetricUserConfig
from grutopia.core.datahub import DataHub


class Env(BaseModel):
    env_id: int
    offset: Optional[List[float]] = None


class TaskRuntime(BaseModel, extra=Extra.allow):
    """
    Runtime task configuration that runs in `grutopia.core.robot.task`.
    """
    type: str
    name: str  # id of task.
    task_idx: str

    # env
    env: Env

    # scene
    scene_asset_path: Optional[str] = None
    scene_scale: Optional[List[float]] = [1.0, 1.0, 1.0]
    scene_position: Optional[List[float]] = [0, 0, 0]
    scene_orientation: Optional[List[float]] = [1.0, 0, 0, 0]

    # inherit
    robots: Optional[List[RobotUserConfig]] = []
    objects: Optional[List[Object]] = []
    metrics: Optional[List[MetricUserConfig]] = []

    # path
    root_path: str
    scene_root_path: Optional[str] = '/scene'
    robots_root_path: Optional[str] = '/robots'
    objects_root_path: Optional[str] = '/objects'

    # task meta info
    meta: Optional[Dict[str, Any]] = {}


def setup_offset_for_assets(episode_dict):
    """
    Set offset for all assets.

    Args:
        episode_dict (dict): All info for an episode.
    """
    env_id = episode_dict['env'].env_id
    offset = episode_dict['env'].offset
    for r in episode_dict['robots']:
        r['name'] = f"{r['name']}_{env_id}"
        r['prim_path'] = episode_dict['root_path'] + episode_dict['robots_root_path'] + r['prim_path']
        r['position'] = [offset[idx] + pos for idx, pos in enumerate(r['position'])]
    if 'objects' in episode_dict and episode_dict['objects'] is not None:
        for o in episode_dict['objects']:
            o['name'] = f"{o['name']}_{env_id}"
            o['prim_path'] = episode_dict['root_path'] + episode_dict['objects_root_path'] + o['prim_path']
            o['position'] = [offset[idx] + pos for idx, pos in enumerate(o['position'])]


class TaskRuntimeManager:
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
        self.env_num = task_user_config.env_num
        self.task_name_prefix = task_user_config.task_name_prefix
        self.task_config: TaskConfig = task_user_config
        self.episodes: List[EpisodeConfig] = task_user_config.episodes
        self.active_runtimes: Dict[str, TaskRuntime] = {}
        self.offset_size: float = self.task_config.offset_size
        self.runtime_template: Dict = self.gen_runtime_template()
        self.init()

    def gen_runtime_template(self):
        task_template = self.task_config.model_dump()
        del task_template['offset_size']
        del task_template['env_num']
        del task_template['episodes']
        del task_template['task_name_prefix']
        return task_template

    def _pop_next_episode(self) -> Union[EpisodeConfig, None]:
        """
        Pop next episode from episodes' list.

        Returns:
            EpisodeConfig: next episode assets. To be assembled to TaskRuntime.
        """
        if self.episodes:
            return self.episodes.pop()
        return None

    def get_next_task_runtime(self, last_env: Env) -> Union[TaskRuntime, None]:
        """
        Get next task runtime with env of last episode.
        Clean last task runtime in active_runtimes.
        Set new task runtime to active_runtimes.

        Args:
            last_env (Env):  Env of last episode.

        Returns:
            TaskRuntime: TaskRuntime for next task.
        """
        next_episode = self._pop_next_episode()
        if next_episode is None:
            del self.active_runtimes[str(last_env.env_id)]
            return None

        assets = next_episode.model_dump()
        next_episode_dict = deepcopy(self.runtime_template)
        next_episode_dict.update(**assets)

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
