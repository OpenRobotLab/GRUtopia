import os
import typing

import yaml
from omni.isaac.core.scenes import Scene

from grutopia.core.robot.robot import BaseRobot, create_robots
from grutopia.core.robot.robot_model import RobotModels
from grutopia.core.runtime.task_runtime import TaskRuntime

# ROBOT_TYPES = {}

ROBOT_MODELS_PATH = os.path.join(
    os.path.split(os.path.realpath(__file__))[0], '../../../grutopia_extension/robots', 'robot_models.yaml')
# print(ROBOT_MODELS_PATH)

with open(ROBOT_MODELS_PATH, 'r') as f:
    models = yaml.load(f.read(), Loader=yaml.FullLoader)
    # print(models)
    robot_models = RobotModels(**models)


def init_robots(runtime: TaskRuntime, scene: Scene) -> typing.Dict[str, BaseRobot]:
    return create_robots(runtime, robot_models, scene)
