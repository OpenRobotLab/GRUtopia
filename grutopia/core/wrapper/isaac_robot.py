from typing import List, Optional, Sequence, Union

import numpy as np
from isaacsim.core.api.controllers.articulation_controller import ArticulationController
from omni.isaac.core.robots.robot import Robot

from grutopia.core.util import log
from grutopia.core.util.physics_status_util import (
    get_articulation_status,
    set_articulation_status,
)
from grutopia.core.util.pose_mixin import PoseMixin


class IsaacRobot(PoseMixin):
    def __init__(
        self,
        prim_path: str,
        name: str = 'robot',
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: bool = True,
        articulation_controller: Optional[ArticulationController] = None,
    ):
        self._robot = Robot(
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            articulation_controller=articulation_controller,
        )
        super().__init__()
        self.status = {}

    def unwrap(self):
        return self._robot

    def save_status(self):
        self.status = get_articulation_status(self._robot)

    def restore_status(self):
        log.info(f'=============== restore info of robot {self.name} ==============')
        if self.status:
            set_articulation_status(self._robot, self.status)

    @property
    def name(self):
        return self._robot.name

    @property
    def prim(self):
        return self._robot.prim

    @property
    def robot_ins(self):
        return self._robot

    @property
    def dof_names(self):
        return self._robot.dof_names

    @property
    def _articulation_view(self):
        return self._robot._articulation_view

    @property
    def handles_initialized(self):
        return self._robot.handles_initialized

    def is_valid(self):
        return self._robot.is_valid()

    def set_solver_position_iteration_count(self, count: int) -> None:
        self._robot.set_solver_position_iteration_count(count)

    def set_solver_velocity_iteration_count(self, count: int) -> None:
        self._robot.set_solver_velocity_iteration_count(count)

    def get_joint_velocities(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        return self._robot.get_joint_velocities(joint_indices)

    def get_joint_positions(self, joint_indices: Optional[Union[List, np.ndarray]] = None) -> np.ndarray:
        return self._robot.get_joint_positions(joint_indices)

    def set_joint_positions(
        self, positions: np.ndarray, joint_indices: Optional[Union[List, np.ndarray]] = None
    ) -> None:
        self._robot.set_joint_positions(positions, joint_indices)

    def get_local_scale(self):
        return self._robot.get_local_scale()

    def get_dof_index(self, dof_name: str) -> int:
        return self._robot.get_dof_index(dof_name)

    def set_enabled_self_collisions(self, flag: bool) -> None:
        self._robot.set_enabled_self_collisions(flag)

    def apply_action(self, control_actions) -> None:
        self._robot.apply_action(control_actions)

    def initialize(self, physics_sim_view=None) -> None:
        self._robot.initialize(physics_sim_view)

    def post_reset(self):
        self._robot.post_reset()

    def get_articulation_controller(self):
        return self._robot.get_articulation_controller()

    @property
    def _articulation_controller(self):
        return self._robot._articulation_controller

    def set_local_scale(self, scale: Optional[Sequence[float]]) -> None:
        self._robot.set_local_scale(scale)
