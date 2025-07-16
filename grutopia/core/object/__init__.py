from collections import OrderedDict

from grutopia.core.config import TaskCfg
from grutopia.core.object.object import BaseObject, create_objects
from grutopia.core.scene.scene import IScene


def init_objects(task_config: TaskCfg, scene: IScene) -> OrderedDict[str, BaseObject]:
    return create_objects(task_config, scene)
