import os
from collections import OrderedDict
from typing import List

import numpy as np
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.robots.robot import Robot as IsaacRobot
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.stage import add_reference_to_stage

import grutopia.core.util.gym as gymutil
from grutopia.core.config.robot import RobotCfg
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.util import log


class G1(IsaacRobot):
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
        joint_names_sim = [
            'left_hip_pitch_joint',
            'right_hip_pitch_joint',
            'waist_yaw_joint',
            'left_hip_roll_joint',
            'right_hip_roll_joint',
            'left_hip_yaw_joint',
            'right_hip_yaw_joint',
            'left_knee_joint',
            'right_knee_joint',
            'left_shoulder_pitch_joint',
            'right_shoulder_pitch_joint',
            'left_ankle_pitch_joint',
            'right_ankle_pitch_joint',
            'left_shoulder_roll_joint',
            'right_shoulder_roll_joint',
            'left_ankle_roll_joint',
            'right_ankle_roll_joint',
            'left_shoulder_yaw_joint',
            'right_shoulder_yaw_joint',
            'left_elbow_joint',
            'right_elbow_joint',
            'left_wrist_roll_joint',
            'right_wrist_roll_joint',
            'left_wrist_pitch_joint',
            'right_wrist_pitch_joint',
            'left_wrist_yaw_joint',
            'right_wrist_yaw_joint',
        ]

        joint_names_gym = [
            'left_hip_pitch_joint',
            'left_hip_roll_joint',
            'left_hip_yaw_joint',
            'left_knee_joint',
            'left_ankle_pitch_joint',
            'left_ankle_roll_joint',
            'right_hip_pitch_joint',
            'right_hip_roll_joint',
            'right_hip_yaw_joint',
            'right_knee_joint',
            'right_ankle_pitch_joint',
            'right_ankle_roll_joint',
            'waist_yaw_joint',
            'left_shoulder_pitch_joint',
            'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint',
            'left_elbow_joint',
            'left_wrist_roll_joint',
            'left_wrist_pitch_joint',
            'left_wrist_yaw_joint',
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_joint',
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'right_wrist_yaw_joint',
        ]
        self.gym_adapter = gymutil.gym_adapter(joint_names_gym, joint_names_sim)

    def set_gains(self):
        """
        Set default stiffness (kps) and damping (kds) for joints.
        """
        self.get_articulation_controller().set_effort_modes('force')
        self.get_articulation_controller().switch_control_mode('effort')

        # num_joints = 27
        kps = self.gym_adapter.gym2sim(
            np.array(
                [
                    150.0,
                    150.0,
                    150.0,
                    300.0,
                    50.0,
                    50.0,
                    150.0,
                    150.0,
                    150.0,
                    300.0,
                    50.0,
                    50.0,
                    300.0,
                    200.0,
                    200.0,
                    200.0,
                    100.0,
                    20.0,
                    20.0,
                    20.0,
                    200.0,
                    200.0,
                    200.0,
                    100.0,
                    20.0,
                    20.0,
                    20.0,
                ]
            )
        )
        kds = self.gym_adapter.gym2sim(
            np.array(
                [
                    2.0000,
                    2.0000,
                    2.0000,
                    4.0000,
                    5.0000,
                    5.0000,
                    2.0000,
                    2.0000,
                    2.0000,
                    4.0000,
                    5.0000,
                    5.0000,
                    5.0000,
                    4.0000,
                    4.0000,
                    4.0000,
                    1.0000,
                    0.5000,
                    0.5000,
                    0.5000,
                    4.0000,
                    4.0000,
                    4.0000,
                    1.0000,
                    0.5000,
                    0.5000,
                    0.5000,
                ]
            )
        )

        if kps is not None:
            kps = self._articulation_view._backend_utils.expand_dims(kps, 0)
        if kds is not None:
            kds = self._articulation_view._backend_utils.expand_dims(kds, 0)
        self._articulation_view.set_gains(kps=kps, kds=kds, save_to_usd=False)


@BaseRobot.register('G1Robot')
class G1Robot(BaseRobot):
    def __init__(self, config: RobotCfg, scene: Scene):
        super().__init__(config, scene)
        self._sensor_config = config.sensors
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'G1 {config.name}: position    : ' + str(self._start_position))
        log.debug(f'G1 {config.name}: orientation : ' + str(self._start_orientation))

        usd_path = config.usd_path

        log.debug(f'G1 {config.name}: usd_path         : ' + str(usd_path))
        log.debug(f'G1 {config.name}: config.prim_path : ' + str(config.prim_path))
        self.isaac_robot = G1(
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

        self._robot_base = RigidPrim(prim_path=config.prim_path + '/pelvis', name=config.name + '_base')
        self._imu_in_torso = RigidPrim(prim_path=config.prim_path + '/imu_link', name=config.name + '_imu_in_torso')

        self._rigid_bodies = [self._robot_base, self._imu_in_torso]
        self.obs_keys = [
            'position',
            'orientation',
            'joint_positions',
            'joint_velocities',
            'controllers',
            'sensors',
        ]

    def post_reset(self):
        super().post_reset()
        self.isaac_robot.set_gains()

    def get_rigid_bodies(self) -> List[RigidPrim]:
        return self._rigid_bodies

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
            'joint_positions': self.isaac_robot.get_joint_positions(),
            'joint_velocities': self.isaac_robot.get_joint_velocities(),
            'controllers': {},
            'sensors': {},
        }

        # common
        for c_obs_name, controller_obs in self.controllers.items():
            obs[c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            obs[sensor_name] = sensor_obs.get_data()
        return OrderedDict((key, obs[key]) for key in self.obs_keys)
