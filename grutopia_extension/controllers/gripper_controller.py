from collections import OrderedDict
from typing import Any, List

import numpy as np
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.types import ArticulationAction

from grutopia.core.robot.controller import BaseController
from grutopia.core.robot.robot import BaseRobot
from grutopia_extension.configs.controllers import GripperControllerCfg


@BaseController.register('GripperController')
class GripperController(BaseController):
    def __init__(self, config: GripperControllerCfg, robot: BaseRobot, scene: Scene):
        self._gripper = robot.isaac_robot.gripper  # for franka is OK
        super().__init__(config, robot, scene)

    def forward(self, action: str) -> ArticulationAction:
        return self._gripper.forward(action)

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """
        Args:
            action (List | np.ndarray): 1-element 1d array.
        """
        assert len(action) == 1 and action[0] in [
            'open',
            'close',
        ], f'action must be a list with one str elem, which is one of "open" / "close", but got {action}'
        return self.forward(action[0])

    def get_obs(self) -> OrderedDict[str, Any]:
        return OrderedDict({'gripper_pos': self._gripper.get_joint_positions()})
