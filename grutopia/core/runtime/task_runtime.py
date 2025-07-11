from typing import Any, Dict, List, Optional, Union

from pydantic import BaseModel, Extra

from grutopia.core.config import Config, EpisodeCfg, ObjectCfg, RobotCfg, TaskCfg
from grutopia.core.config.metric import MetricCfg
from grutopia.core.config.task.reward import RewardCfg


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

    # custom setting
    task_settings: Optional[Any] = None
    metrics_save_path: Optional[str] = 'console'

    # scene
    scene_asset_path: Optional[str] = None
    scene_scale: Optional[List[float]] = [1.0, 1.0, 1.0]
    scene_position: Optional[List[float]] = [0, 0, 0]
    scene_orientation: Optional[List[float]] = [1.0, 0, 0, 0]

    # inherit
    robots: Optional[List[RobotCfg]] = []
    objects: Optional[List[ObjectCfg]] = []
    metrics: Optional[List[MetricCfg]] = []
    reward_setting: Optional[RewardCfg] = []

    # path
    root_path: str
    scene_root_path: Optional[str] = '/scene'
    robots_root_path: Optional[str] = '/robots'
    objects_root_path: Optional[str] = '/objects'

    # task extra info
    extra: Optional[Any] = None


def setup_offset_for_assets(
    episode: EpisodeCfg, env: Env, root_path: str, robots_root_path: str, objects_root_path: str
):
    env_id = env.env_id
    offset = env.offset
    for r in episode.robots:
        r.name = f'{r.name}_{env_id}'
        r.prim_path = root_path + robots_root_path + r.prim_path
        r.position = [offset[idx] + pos for idx, pos in enumerate(r.position)]
    if episode.objects is not None:
        for o in episode.objects:
            o.name = f'{o.name}_{env_id}'
            o.prim_path = root_path + objects_root_path + o.prim_path
            o.position = [offset[idx] + pos for idx, pos in enumerate(o.position)]


class BaseTaskRuntimeManager:
    """Base class of task runtime manager.

    Attributes:
        env_num (int): Env number.
        task_config (TaskConfig): Task config that user input.
        episodes (List[EpisodeConfig]): Rest episodes.
        active_runtimes (str): Activated runtimes, format like => { env_id: TaskRuntime }.
        offset_size (float): Offset size.
        all_allocated (bool): true if all tasks are allocated.
    """

    managers = {}

    def __init__(self, config: Config = None):
        self.task_config: TaskCfg = config.task_config
        self.episodes: List[EpisodeCfg] = self.task_config.episodes

        # validate env_num
        self.env_num = self.task_config.env_num
        if self.env_num <= 0:
            raise RuntimeError('env_num must be greater than 0')

        # validate episodes
        episodes_count = self.validate_episodes()
        if episodes_count == 0:
            raise RuntimeError('len(episodes) must be greater than 0')

        self.active_runtimes: Dict[int, TaskRuntime] = {}
        self.offset_size: float = self.task_config.offset_size
        self.runtime_template: Dict = self.gen_runtime_template()
        self.all_allocated = False

    def validate_episodes(self):
        episodes_count = 0
        _order = None
        for episode in self.episodes:
            episodes_count += 1
            robot_order = [robot.type for robot in episode.robots]
            if _order is None:
                _order = robot_order
                continue
            if robot_order != _order:
                raise ValueError('robot types must be identical across all episodes')
        return episodes_count

    def gen_runtime_template(self):
        task_template = {}
        #  Dump only the first level of the model. model_dump will dump recursively and lose type information.
        for k in self.task_config.model_dump():
            task_template[k] = getattr(self.task_config, k)
        del task_template['offset_size']
        del task_template['env_num']
        del task_template['episodes']
        del task_template['metrics_save_path']
        return task_template

    def active_runtime(self) -> Dict[int, TaskRuntime]:
        """
        Get active runtimes with env id as key.
        """
        return self.active_runtimes

    def get_next_task_runtime(self, last_env: Optional[Env] = None) -> Union[TaskRuntime, None]:
        """
        Get next task runtime with env of last episode.
        Args:
            last_env (Env): Env of last episode.

        Returns:
            TaskRuntime: TaskRuntime for next task.
        """
        raise NotImplementedError()


def create_task_runtime_manager(config: Config):
    from .local_task_runtime_manager import LocalTaskRuntimeManager

    return LocalTaskRuntimeManager(config)
