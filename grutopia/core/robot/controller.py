# yapf: disable
from abc import ABC, abstractmethod
from functools import wraps
from typing import Any, Dict, List, Union

import numpy as np
from omni.isaac.core.articulations import ArticulationSubset
from omni.isaac.core.controllers import BaseController as Base
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.types import ArticulationAction

from grutopia.core.config.robot import RobotUserConfig
from grutopia.core.config.robot.params import ControllerParams
from grutopia.core.robot.robot import BaseRobot

# yapf: disable
from grutopia.core.robot.robot_model import ControllerModel, RobotModel

# yapf: enable
from grutopia.core.util import log

# yapf: enable


class BaseController(Base, ABC):
    """Base class of controller."""

    controllers = {}

    def __init__(self, config: ControllerModel, robot: BaseRobot, scene: Scene):
        """Initialize the controller.

        Args:
            config (ControllerModel): merged config (from user config and robot model) of the controller.
            robot (BaseRobot): robot owning the controller.
            scene (Scene): scene from isaac sim.

        """
        self.scene = scene
        if config.name is None:
            raise ValueError('must specify controller name.')
        super().__init__(config.name)
        self._obs = {}
        self._robot = robot
        self.config = config
        self.sub_controllers: List[BaseController]

    @abstractmethod
    def action_to_control(self, action: Union[np.ndarray, List]) -> ArticulationAction:
        """Convert input action (in 1d array format) to joint signals to apply.

        Args:
            action (Union[np.ndarray, List]): input control action.

        Returns:
            ArticulationAction: joint signals to apply
        """
        raise NotImplementedError()

    def get_obs(self) -> Dict[str, Any]:
        """Get observation of controller.

        Returns:
            Dict[str, Any]: observation key and value.
        """
        obs = {}
        for key, obs_ins in self._obs.items():
            obs[key] = obs_ins.get_obs()
        return obs

    @classmethod
    def register(cls, name: str):
        """Register a controller with its name(decorator).

        Args:
            name (str): name of the controller
        """

        def decorator(controller_class):
            cls.controllers[name] = controller_class

            @wraps(controller_class)
            def wrapped_function(*args, **kwargs):
                return controller_class(*args, **kwargs)

            return wrapped_function

        return decorator

    @property
    def robot(self):
        return self._robot

    @robot.setter
    def robot(self, value):
        self._robot = value

    def cleanup(self):
        """
        Operations that need to be cleaned up before switching scenes (or resetting)
        """
        pass

    def get_joint_subset(self) -> ArticulationSubset:
        """Get the joint subset controlled by the controller.

        Returns:
            ArticulationSubset: joint subset.
        """
        if hasattr(self, 'joint_subset'):
            return self.joint_subset
        if not hasattr(self, 'sub_controllers'):
            return None
        if self.sub_controllers is None or len(self.sub_controllers) == 0:
            return None
        return self.sub_controllers[0].get_joint_subset()


def config_inject(user_config: ControllerParams, model: ControllerModel) -> ControllerModel:
    """Merge controller config from user config and robot model.

    Args:
        user_config (ControllerParams): user config.
        model (ControllerModel): controller config from robot model.

    Returns:
        ControllerModel: merged controller config.
    """
    config = model.dict()
    user = user_config.dict()
    for k, v in user.items():
        if v is not None:
            config[k] = v
    conf = ControllerModel(**config)

    return conf


def create_controllers(
    config: RobotUserConfig, robot_model: RobotModel, robot: BaseRobot, scene: Scene
) -> Dict[str, BaseController]:
    """Create all controllers of one robot.

    Args:
        config (RobotUserConfig): user config of the robot.
        robot_model (RobotModel): model of the robot.
        robot (BaseRobot): robot instance.
        scene (Scene): scene from isaac sim.

    Returns:
        Dict[str, BaseController]: dict of controllers with controller name as key.
    """
    controller_map = {}
    available_controllers = {a.name: a for a in robot_model.controllers}

    if config.controller_params is None:
        return controller_map
    for controller_param in config.controller_params:
        controller_name = controller_param.name
        if controller_name in available_controllers:
            controller_config = config_inject(controller_param, available_controllers[controller_name])
            controller_cls = BaseController.controllers[controller_config.type]
            controller_ins: BaseController = controller_cls(config=controller_config, robot=robot, scene=scene)
            if controller_config.sub_controllers is not None:
                inject_sub_controllers(
                    parent=controller_ins,
                    configs=controller_config.sub_controllers,
                    available=available_controllers,
                    robot=robot,
                    scene=scene,
                )
        else:
            log.debug(available_controllers)
            raise KeyError(f'{controller_name} not registered in controllers of {config.type}')

        controller_map[controller_name] = controller_ins
        log.debug(f'==================== {controller_name} loaded==========================')

    return controller_map


def inject_sub_controllers(
    parent: BaseController,
    configs: List[ControllerParams],
    available: Dict[str, ControllerModel],
    robot: BaseRobot,
    scene: Scene,
):
    """Recursively create and inject sub-controlllers into parent controller.

    Args:
        parent (BaseController): parent controller instance.
        configs (List[ControllerParams]): user configs of sub-controllers.
        available (Dict[str, ControllerModel]): available controllers.
        robot (BaseRobot): robot instance.
        scene (Scene): scene from isaac sim.
    """
    if len(configs) == 0:
        return
    sub_controllers: List[BaseController] = []
    for config in configs:
        controller_name = config.name
        if controller_name not in available:
            raise KeyError(f'{controller_name} not registered in controllers of {robot.robot_model.type}')
        controller_config = config_inject(config, available[controller_name])
        controller_cls = BaseController.controllers[controller_config.type]
        controller_ins = controller_cls(config=controller_config, robot=robot, scene=scene)
        if controller_config.sub_controllers is not None:
            inject_sub_controllers(
                controller_ins, configs=controller_config.sub_controllers, available=available, robot=robot, scene=scene
            )
        sub_controllers.append(controller_ins)

    parent.sub_controllers = sub_controllers
