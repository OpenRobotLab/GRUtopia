from collections import OrderedDict

from grutopia.core.robot.robot import BaseRobot, create_robots
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.scene.scene import IScene


def init_robots(runtime: TaskRuntime, scene: IScene) -> OrderedDict[str, BaseRobot]:
    return create_robots(runtime, scene)
