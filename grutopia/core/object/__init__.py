from collections import OrderedDict

from grutopia.core.object.object import BaseObject, create_objects
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.scene.scene import IScene


def init_objects(runtime: TaskRuntime, scene: IScene) -> OrderedDict[str, BaseObject]:
    return create_objects(runtime, scene)
