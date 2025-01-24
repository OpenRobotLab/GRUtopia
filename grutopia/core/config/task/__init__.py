from typing import Any, Dict, List, Literal, Optional, Union

from pydantic import BaseModel, Extra

from grutopia.core.config.metric import MetricUserConfig
from grutopia.core.config.task.episode import EpisodeCfg
from grutopia.core.config.task.reward import RewardConfig


class TaskCfg(BaseModel, extra=Extra.allow):
    """
    A configuration model for defining tasks, including their environment settings, paths, metrics, and execution modes.

    This class extends `BaseModel` from the Pydantic library and allows additional fields not explicitly defined.
    It centralizes the configuration parameters necessary to set up and control how tasks are executed within a simulation
    or other computational environment.

    Attributes:
        task_name_prefix (str): A prefix to be used when naming tasks, enhancing identification and organization.

        env_num (Optional[int], default=1): Specifies the number of environments to be instantiated for parallel execution.

        metrics (Optional[List[MetricUserConfig]], default=[]): Configuration details for metrics to track during task execution.

        metrics_save_path (Optional[str], default='console'): Determines where metric results are saved; defaults to console output.

        scene_root_path (Optional[str], default='/scene'): Root directory path for scene configurations.

        robots_root_path (Optional[str], default='/robots'): Root directory path for robot configurations.

        objects_root_path (Optional[str], default='/objects'): Root directory path for object configurations.

        task_settings (Optional[Dict], default={}): Custom settings specific to the task, allowing flexibility for unique requirements.

        offset_size (Optional[float], default=10.0): Defines an offset size, potentially for spacing in the simulated environment.

        episodes (List[EpisodeConfig]): Configures the episodes to be executed. Should be a list of EpisodeConfig instances.

        operation_mode (Literal['local', 'distributed'], default='local'): Specifies the operational mode as either local or distributed computing.

        loop (Optional[bool], default=False): When True, the first episode will repeat indefinitely instead of moving to the next episode.
    """

    type: str
    env_num: Optional[int] = 1

    # inherit
    metrics: Optional[List[MetricUserConfig]] = []
    metrics_save_path: Optional[str] = 'console'
    reward_setting: Optional[RewardConfig] = None

    # path
    scene_root_path: Optional[str] = '/scene'
    robots_root_path: Optional[str] = '/robots'
    objects_root_path: Optional[str] = '/objects'

    # custom setting
    task_settings: Optional[Any] = None

    # offset
    offset_size: Optional[float] = 10.0

    # episode
    episodes: List[EpisodeCfg]

    # Operation_mode is used to specify the execution environment for the task.
    # It accepts one of the following values:
    # - `local`: Indicates that the task will be executed on a single compute node.
    # - `distributed`: Indicates that the task will be executed across multiple compute nodes.
    operation_mode: Literal['local', 'distributed'] = 'local'
    # loop: Keep repeating the first episode.
    # (reset means next_episode)
    loop: Optional[bool] = False
