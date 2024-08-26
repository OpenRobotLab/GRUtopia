from typing import List, Optional, Union

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

    # path
    scene_root_path: Optional[str] = '/scene'
    robots_root_path: Optional[str] = '/robots'
    objects_root_path: Optional[str] = '/objects'

    # offset
    offset_size: Optional[float] = 10.0

    # episode
    episodes: Union[List[EpisodeConfig], str]
