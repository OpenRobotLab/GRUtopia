from typing import Dict, List, Literal, Optional, Union

from pydantic import BaseModel, Extra

from grutopia.core.config.metric import MetricUserConfig
from grutopia.core.config.task.episode import EpisodeConfig


class TaskConfig(BaseModel, extra=Extra.allow):
    """
    Task config that user input.
    """

    type: str
    task_name_prefix: str
    env_num: Optional[int] = 1

    # inherit
    metrics: Optional[List[MetricUserConfig]] = []
    metrics_save_path: Optional[str] = 'console'

    # path
    scene_root_path: Optional[str] = '/scene'
    robots_root_path: Optional[str] = '/robots'
    objects_root_path: Optional[str] = '/objects'

    # custom setting
    task_settings: Optional[Dict] = {}

    # offset
    offset_size: Optional[float] = 10.0

    # episode
    episodes: Union[List[EpisodeConfig], str]

    # Operation_mode is used to specify the execution environment for the task.
    # It accepts one of the following values:
    # - `local`: Indicates that the task will be executed on a single compute node.
    # - `distributed`: Indicates that the task will be executed across multiple compute nodes.
    operation_mode: Literal['local', 'distributed'] = 'local'
