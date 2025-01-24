from typing import List

import numpy as np
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.types import ArticulationAction

from grutopia.core.robot.controller import BaseController
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.util import log
from grutopia_extension.configs.controllers import RecoverControllerCfg


@BaseController.register('RecoverController')
class RecoverController(BaseController):
    """Controller for recovering from pose failure."""

    def __init__(self, config: RecoverControllerCfg, robot: BaseRobot, scene: Scene) -> None:
        super().__init__(config=config, robot=robot, scene=scene)

        self.recover_height = config.recover_height
        self.target_position = None
        self.num_joints = None

    def forward(
        self, target_position: List | np.ndarray, target_orientation: List | np.ndarray = np.array([1.0, 0.0, 0.0, 0.0])
    ) -> ArticulationAction:
        if self.num_joints is None:
            self.num_joints = self.get_joint_subset().num_joints
        current_position = self.robot.get_world_pose()[0]
        log.info(f'current pos is {current_position}, recovering to {target_position}')
        self.robot.isaac_robot.set_world_pose(target_position, target_orientation)
        self.robot.isaac_robot.set_world_velocity(np.zeros(6))

        self.robot.isaac_robot.set_joint_velocities(np.zeros(len(self.robot.isaac_robot.dof_names)))
        self.robot.isaac_robot.set_joint_positions(np.zeros(len(self.robot.isaac_robot.dof_names)))
        self.robot.isaac_robot.set_linear_velocity(np.zeros(3))

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
        # assert len(action) == 0, 'action must be empty'
        if len(action) > 1:
            return self.forward(action[0], action[1])

        return self.forward(action[0])
