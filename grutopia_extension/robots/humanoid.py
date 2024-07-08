import os
from typing import Dict

import numpy as np
import torch
from omni.isaac.core.articulations import ArticulationSubset
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.robots.robot import Robot as IsaacRobot
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction, ArticulationActions

import grutopia.core.util.string as string_utils
from grutopia.actuators import ActuatorBase, ActuatorBaseCfg, DCMotorCfg
from grutopia.core.config.robot import RobotUserConfig as Config
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.robot.robot_model import RobotModel
from grutopia.core.util import log


class Humanoid(IsaacRobot):

    actuators_cfg = {
        'base_legs':
        DCMotorCfg(
            joint_names_expr=['.*'],
            effort_limit={
                '.*hip.*': 200,
                '.*knee.*': 300,
                '.*ankle.*': 40,
                'torso_joint': 200,
                '.*shoulder_pitch.*': 40,
                '.*shoulder_roll.*': 40,
                '.*shoulder_yaw.*': 18,
                '.*elbow.*': 18
            },
            saturation_effort=400.0,
            velocity_limit=1000.0,
            stiffness={
                '.*hip.*': 200,
                '.*knee.*': 300,
                '.*ankle.*': 40,
                'torso_joint': 300,
                '.*shoulder.*': 100,
                '.*elbow.*': 100
            },
            damping={
                '.*hip.*': 5,
                '.*knee.*': 6,
                '.*ankle.*': 2,
                'torso_joint': 6,
                '.*shoulder.*': 2,
                '.*elbow.*': 2
            },
            friction=0.0,
            armature=0.0,
        ),
    }

    def __init__(self,
                 prim_path: str,
                 usd_path: str,
                 name: str,
                 position: np.ndarray = None,
                 orientation: np.ndarray = None,
                 scale: np.ndarray = None):
        add_reference_to_stage(prim_path=prim_path, usd_path=os.path.abspath(usd_path))
        super().__init__(prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale)
        self.actuators: Dict[str, ActuatorBase]

    def set_gains(self, gains):
        """[summary]

        Args:
            kps (Optional[np.ndarray], optional): [description]. Defaults to None.
            kds (Optional[np.ndarray], optional): [description]. Defaults to None.

        Raises:
            Exception: [description]
        """
        num_leg_joints = 19
        kps = np.array([0.] * num_leg_joints)
        kds = np.array([0.] * num_leg_joints)

        if kps is not None:
            kps = self._articulation_view._backend_utils.expand_dims(kps, 0)
        if kds is not None:
            kds = self._articulation_view._backend_utils.expand_dims(kds, 0)
        self._articulation_view.set_gains(kps=kps, kds=kds, save_to_usd=False)
        # VERY important!!! additional physics parameter
        self._articulation_view.set_solver_position_iteration_counts(
            self._articulation_view._backend_utils.expand_dims(4, 0))
        self._articulation_view.set_solver_velocity_iteration_counts(
            self._articulation_view._backend_utils.expand_dims(0, 0))
        self._articulation_view.set_enabled_self_collisions(self._articulation_view._backend_utils.expand_dims(True, 0))

    def _process_actuators_cfg(self):
        self.actuators = dict.fromkeys(Humanoid.actuators_cfg.keys())
        for actuator_name, actuator_cfg in Humanoid.actuators_cfg.items():
            # type annotation for type checkersc
            actuator_cfg: ActuatorBaseCfg
            # create actuator group
            joint_ids, joint_names = self.find_joints(actuator_cfg.joint_names_expr)
            stiffness, damping = self._articulation_view.get_gains()
            actuator: ActuatorBase = actuator_cfg.class_type(
                cfg=actuator_cfg,
                joint_names=joint_names,
                joint_ids=joint_ids,
                num_envs=1,
                device='cpu',
                stiffness=self._articulation_view.get_gains()[0][0],
                damping=self._articulation_view.get_gains()[1][0],
                armature=torch.tensor(self._articulation_view.get_armatures()),
                friction=torch.tensor(self._articulation_view.get_friction_coefficients()),
                effort_limit=torch.tensor(self._articulation_view._physics_view.get_dof_max_forces()),
                velocity_limit=torch.tensor(self._articulation_view._physics_view.get_dof_max_velocities()),
            )
            # log information on actuator groups
            self.actuators[actuator_name] = actuator

    def apply_actuator_model(self, control_action: ArticulationAction, controller_name: str,
                             joint_set: ArticulationSubset):
        name = 'base_legs'
        actuator = self.actuators[name]

        control_joint_pos = torch.tensor(control_action.joint_positions, dtype=torch.float32)
        control_actions = ArticulationActions(
            joint_positions=control_joint_pos,
            joint_velocities=torch.zeros_like(control_joint_pos),
            joint_efforts=torch.zeros_like(control_joint_pos),
            joint_indices=actuator.joint_indices,
        )

        joint_pos = torch.tensor(self.get_joint_positions(), dtype=torch.float32)
        joint_vel = torch.tensor(self.get_joint_velocities(), dtype=torch.float32)
        control_actions = actuator.compute(
            control_actions,
            joint_pos=joint_pos,
            joint_vel=joint_vel,
        )
        if control_actions.joint_positions is not None:
            joint_set.set_joint_positions(control_actions.joint_positions)
        if control_actions.joint_velocities is not None:
            joint_set.set_joint_velocities(control_actions.joint_velocities)
        if control_actions.joint_efforts is not None:
            self._articulation_view._physics_view.set_dof_actuation_forces(control_actions.joint_efforts,
                                                                           torch.tensor([0]))

    def find_joints(self, name_keys, joint_subset=None):
        """Find joints in the articulation based on the name keys.

        Please see the :func:`omni.isaac.orbit.utils.string.resolve_matching_names` function for more information
        on the name matching.

        Args:
            name_keys: A regular expression or a list of regular expressions to match the joint names.
            joint_subset: A subset of joints to search for. Defaults to None, which means all joints
                in the articulation are searched.

        Returns:
            A tuple of lists containing the joint indices and names.
        """
        if joint_subset is None:
            joint_subset = self._articulation_view.dof_names
        # find joints
        return string_utils.resolve_matching_names(name_keys, joint_subset)


@BaseRobot.register('HumanoidRobot')
class HumanoidRobot(BaseRobot):

    def __init__(self, config: Config, robot_model: RobotModel, scene: Scene):
        super().__init__(config, robot_model, scene)
        self._sensor_config = robot_model.sensors
        self._gains = robot_model.gains
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'humanoid {config.name}: position    : ' + str(self._start_position))
        log.debug(f'humanoid {config.name}: orientation : ' + str(self._start_orientation))

        usd_path = robot_model.usd_path
        if usd_path.startswith('/Isaac'):
            usd_path = get_assets_root_path() + usd_path

        log.debug(f'humanoid {config.name}: usd_path         : ' + str(usd_path))
        log.debug(f'humanoid {config.name}: config.prim_path : ' + str(config.prim_path))
        self.isaac_robot = Humanoid(
            prim_path=config.prim_path,
            name=config.name,
            position=self._start_position,
            orientation=self._start_orientation,
            usd_path=usd_path,
        )

        if robot_model.joint_names is not None:
            self.joint_subset = ArticulationSubset(self.isaac_robot, robot_model.joint_names)

        self._robot_scale = np.array([1.0, 1.0, 1.0])
        if config.scale is not None:
            self._robot_scale = np.array(config.scale)
            self.isaac_robot.set_local_scale(self._robot_scale)

        self._robot_ik_base = None

        self._robot_base = RigidPrim(prim_path=config.prim_path + '/pelvis', name=config.name + '_base')

    def post_reset(self):
        super().post_reset()
        self.isaac_robot._process_actuators_cfg()
        if self._gains is not None:
            self.isaac_robot.set_gains(self._gains)

    def get_robot_scale(self):
        return self._robot_scale

    def get_robot_base(self) -> RigidPrim:
        return self._robot_base

    def get_robot_ik_base(self):
        return self._robot_ik_base

    def get_world_pose(self):
        return self._robot_base.get_world_pose()

    def apply_action(self, action: dict):
        """
        Args:
            action (dict): inputs for controllers.
        """
        for controller_name, controller_action in action.items():
            if controller_name not in self.controllers:
                log.warn(f'unknown controller {controller_name} in action')
                continue
            controller = self.controllers[controller_name]
            control = controller.action_to_control(controller_action)
            self.isaac_robot.apply_actuator_model(control, controller_name, self.joint_subset)

    def get_obs(self):
        position, orientation = self._robot_base.get_world_pose()

        # custom
        obs = {
            'position': position,
            'orientation': orientation,
        }

        # common
        for c_obs_name, controller_obs in self.controllers.items():
            obs[c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            obs[sensor_name] = sensor_obs.get_data()
        return obs
