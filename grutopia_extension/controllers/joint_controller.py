from typing import List

import numpy as np
from omni.isaac.core.articulations import ArticulationSubset
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.types import ArticulationAction

from grutopia.core.robot.controller import BaseController
from grutopia.core.robot.robot import BaseRobot
from grutopia_extension.configs.controllers import JointControllerCfg


@BaseController.register('JointController')
class JointController(BaseController):
    """Controller for direct joint control."""

    def __init__(self, config: JointControllerCfg, robot: BaseRobot, scene: Scene) -> None:
        super().__init__(config=config, robot=robot, scene=scene)

        self.joint_subset = None
        self.joint_names = config.joint_names
        if self.joint_names is not None:
            self.joint_subset = ArticulationSubset(self.robot.isaac_robot, self.joint_names)

    def forward(self, joint_positions: np.ndarray = None, joint_velocities: np.ndarray = None) -> ArticulationAction:
        if self.joint_subset is None:
            return ArticulationAction(joint_positions=joint_positions, joint_velocities=joint_velocities)

        return self.joint_subset.make_articulation_action(
            joint_positions=joint_positions, joint_velocities=joint_velocities
        )

    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """Convert input action (in 1d array format) to joint signals to apply.

        Args:
            action (List | np.ndarray): 2-element 1d array containing:
              0. joint_positions (np.ndarray)
              1. joint_velocities (np.ndarray) (optional)

        Returns:
            ArticulationAction: joint signals to apply.
        """
        if len(action) == 0:
            return ArticulationAction()
        if len(action) == 1:
            joint_pos = np.array(action[0]) if action[0] is not None else None
            joint_vel = None
        if len(action) > 1:
            joint_pos = np.array(action[0]) if action[0] is not None else None
            joint_vel = np.array(action[1]) if action[1] is not None else None
            if joint_pos is None:
                joint_pos = (
                    self.robot.isaac_robot.get_joint_positions()
                    if self.joint_subset is None
                    else self.joint_subset.get_joint_positions()
                )

        return self.forward(joint_positions=joint_pos, joint_velocities=joint_vel)
