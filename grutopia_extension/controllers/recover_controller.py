from typing import List

import numpy as np
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.types import ArticulationAction

from grutopia.core.robot.controller import BaseController
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.robot.robot_model import ControllerModel
from grutopia.core.util import log


@BaseController.register('RecoverController')
class RecoverController(BaseController):
    """Controller for recovering from pose failure."""

    def __init__(self, config: ControllerModel, robot: BaseRobot, scene: Scene) -> None:
        super().__init__(config=config, robot=robot, scene=scene)

        if config.recover_height is None:
            raise ValueError('Recover controller requires a recover_height parameter')
        self.recover_height = config.recover_height
        self.target_position = None
        self.num_joints = None

    def forward(self) -> ArticulationAction:
        if self.num_joints is None:
            self.num_joints = self.get_joint_subset().num_joints
        current_position = self.robot.get_world_pose()[0]
        error = 1.5  # a big enough default value.
        if self.target_position is not None:
            error = np.linalg.norm(current_position - self.target_position)
        if error > 1.0:
            self.target_position = current_position + np.array([0.0, 0.0, self.recover_height])
            log.info(f'current pos is {current_position}, recovering to {self.target_position}')
        self.robot.isaac_robot.set_world_pose(self.target_position, np.array([1.0, 0.0, 0.0, 0.0]))
        self.robot.isaac_robot.set_world_velocity(np.zeros(6))
        return self.sub_controllers[0].forward(
            joint_positions=np.zeros(self.num_joints),
            joint_velocities=np.zeros(self.num_joints),
        )

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """Convert input action (in 1d array format) to joint signals to apply.

        Args:
            action (List | np.ndarray): 0-element 1d array.

        Returns:
            ArticulationAction: joint signals to apply.
        """
        assert len(action) == 0, 'action must be empty'
        return self.forward()
