import os
import typing

import yaml
from omni.isaac.core.scenes import Scene

from grutopia.core.config import TaskUserConfig
from grutopia.core.robot.robot import create_robots
from grutopia.core.robot.robot_model import RobotModels

# ROBOT_TYPES = {}

ROBOT_MODELS_PATH = os.path.join(
    os.path.split(os.path.realpath(__file__))[0], '../../../grutopia_extension/robots', 'robot_models.yaml')
# print(ROBOT_MODELS_PATH)

with open(ROBOT_MODELS_PATH, 'r') as f:
    models = yaml.load(f.read(), Loader=yaml.FullLoader)
    # print(models)
    robot_models = RobotModels(**models)


def init_robots(the_config: TaskUserConfig, scene: Scene) -> typing.Dict:
    return create_robots(the_config, robot_models, scene)
