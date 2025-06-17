import os
from collections import OrderedDict

import numpy as np
from omni.isaac.core.articulations import ArticulationSubset
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.robots.robot import Robot as IsaacRobot
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.stage import add_reference_to_stage

from grutopia.core.config.robot import RobotCfg
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.util import log


class Aliengo(IsaacRobot):
    def __init__(
        self,
        prim_path: str,
        usd_path: str,
        name: str,
        position: np.ndarray = None,
        orientation: np.ndarray = None,
        scale: np.ndarray = None,
    ):
        add_reference_to_stage(prim_path=prim_path, usd_path=os.path.abspath(usd_path))
        super().__init__(prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale)

    def set_gains(self):
        """
        Set default stiffness (kps) and damping (kds) for joints.
        """
        num_leg_joints = 12
        kps = np.array([40.0] * num_leg_joints)
        kds = np.array([2.0] * num_leg_joints)

        joint_names = [
            'FL_hip_joint',
            'FR_hip_joint',
            'RL_hip_joint',
            'RR_hip_joint',
            'FL_thigh_joint',
            'FR_thigh_joint',
            'RL_thigh_joint',
            'RR_thigh_joint',
            'FL_calf_joint',
            'FR_calf_joint',
            'RL_calf_joint',
            'RR_calf_joint',
        ]
        joint_subset = ArticulationSubset(self, joint_names)

        if kps is not None:
            kps = self._articulation_view._backend_utils.expand_dims(kps, 0)
        if kds is not None:
            kds = self._articulation_view._backend_utils.expand_dims(kds, 0)
        self._articulation_view.set_gains(kps=kps, kds=kds, save_to_usd=False, joint_indices=joint_subset.joint_indices)

        # VERY IMPORTANT!!! additional physics parameter
        self._articulation_view.set_solver_position_iteration_counts(
            self._articulation_view._backend_utils.expand_dims(8, 0)
        )
        self._articulation_view.set_solver_velocity_iteration_counts(
            self._articulation_view._backend_utils.expand_dims(0, 0)
        )
        self._articulation_view.set_enabled_self_collisions(self._articulation_view._backend_utils.expand_dims(True, 0))


@BaseRobot.register('AliengoRobot')
class AliengoRobot(BaseRobot):
    def __init__(self, config: RobotCfg, scene: Scene):
        super().__init__(config, scene)
        self._sensor_config = config.sensors
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'aliengo {config.name}: position    : ' + str(self._start_position))
        log.debug(f'aliengo {config.name}: orientation : ' + str(self._start_orientation))

        usd_path = config.usd_path

        log.debug(f'aliengo {config.name}: usd_path         : ' + str(usd_path))
        log.debug(f'aliengo {config.name}: config.prim_path : ' + str(config.prim_path))
        self.isaac_robot = Aliengo(
            prim_path=config.prim_path,
            name=config.name,
            position=self._start_position,
            orientation=self._start_orientation,
            usd_path=usd_path,
        )

        self._robot_scale = np.array([1.0, 1.0, 1.0])
        if config.scale is not None:
            self._robot_scale = np.array(config.scale)
            self.isaac_robot.set_local_scale(self._robot_scale)

    def _set_rigid_bodies(self):
        self._robot_base = self._rigid_body_map[self.config.prim_path + '/base']

    def restore_robot_info(self):
        super().restore_robot_info()
        self._set_rigid_bodies()
        self.isaac_robot.set_gains()

    def post_reset(self):
        super().post_reset()
        self._set_rigid_bodies()
        self.isaac_robot.set_gains()

    def get_robot_scale(self):
        return self._robot_scale

    def get_robot_base(self) -> RigidPrim:
        return self._robot_base

    def get_world_pose(self):
        return self._robot_base.get_world_pose()

    def apply_action(self, action: dict):
        """
        Args:
            action (dict): inputs for controllers.
        """
        for controller_name, controller_action in action.items():
            if controller_name not in self.controllers:
                log.warning(f'unknown controller {controller_name} in action')
                continue
            controller = self.controllers[controller_name]
            control = controller.action_to_control(controller_action)
            self.isaac_robot.apply_action(control)

    def get_obs(self) -> OrderedDict:
        position, orientation = self._robot_base.get_world_pose()

        # custom
        obs = {
            'position': position,
            'orientation': orientation,
            'controllers': {},
            'sensors': {},
        }

        # common
        for c_obs_name, controller_obs in self.controllers.items():
            obs['controllers'][c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            obs['sensors'][sensor_name] = sensor_obs.get_data()
        return self._make_ordered(obs)
