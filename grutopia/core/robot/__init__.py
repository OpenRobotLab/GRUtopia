import typing

from omni.isaac.core.scenes import Scene

from grutopia.core.robot.robot import BaseRobot, create_robots
from grutopia.core.runtime.task_runtime import TaskRuntime


def init_robots(runtime: TaskRuntime, scene: Scene) -> typing.Dict[str, BaseRobot]:
    return create_robots(runtime, scene)
