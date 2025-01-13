from typing import List

import numpy as np
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.types import ArticulationAction

from grutopia.core.robot.controller import BaseController
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.robot.robot_model import ControllerModel
from grutopia.core.util.interaction import BaseInteraction


@BaseController.register('MoveWithKeyboardController')
class MoveWithKeyboardController(BaseController):
    """Controller for locomotion with keyboard control by utilizing a move-by-speed controller as sub-controller.

    Keyboard mappings:
        W: forward
        S: backward
        A: left
        D: right
        Q: turn left
        E: turn right
    """

    def __init__(self, config: ControllerModel, robot: BaseRobot, scene: Scene) -> None:
        self.config = config
        self.forward_speed_base = config.forward_speed
        self.rotation_speed_base = config.rotation_speed
        self.lateral_speed_base = config.lateral_speed

        self.keyboard: BaseInteraction = BaseInteraction.interactions['Keyboard']()

        super().__init__(config=config, robot=robot, scene=scene)

    def forward(self) -> ArticulationAction:
        key_input = self.keyboard.get_input()
        if len(key_input) != 6:
            raise RuntimeError(f'Please update the keyboard interactive. The key_input is {key_input}')
        forward_speed = 0
        lateral_speed = 0
        rotation_speed = 0

        if key_input[0] == 1:
            forward_speed = 1
        elif key_input[1] == 1:
            forward_speed = -1
        elif key_input[2] == 1:
            lateral_speed = 1
        elif key_input[3] == 1:
            lateral_speed = -1
        elif key_input[4] == 1:
            rotation_speed = 1
        elif key_input[5] == 1:
            rotation_speed = -1

        forward_speed *= self.forward_speed_base
        rotation_speed *= self.rotation_speed_base
        lateral_speed *= self.lateral_speed_base

        return self.sub_controllers[0].forward(
            forward_speed=forward_speed, rotation_speed=rotation_speed, lateral_speed=lateral_speed
        )

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """
        Args:
            action (List | np.ndarray): 0-element 1d array.
        """
        assert len(action) == 0, 'action must be empty'
        return self.forward()
