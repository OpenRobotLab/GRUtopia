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


class GR1(IsaacRobot):
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
        # DEBUG: load saved torques
        self._i = 0
        joint_names_sim = [
            'head_roll_joint',
            'left_shoulder_pitch_joint',
            'right_shoulder_pitch_joint',
            'waist_roll_joint',
            'head_pitch_joint',
            'left_shoulder_roll_joint',
            'right_shoulder_roll_joint',
            'waist_pitch_joint',
            'head_yaw_joint',
            'left_shoulder_yaw_joint',
            'right_shoulder_yaw_joint',
            'waist_yaw_joint',
            'left_elbow_pitch_joint',
            'right_elbow_pitch_joint',
            'left_hip_roll_joint',
            'right_hip_roll_joint',
            'left_wrist_yaw_joint',
            'right_wrist_yaw_joint',
            'left_hip_yaw_joint',
            'right_hip_yaw_joint',
            'left_wrist_roll_joint',
            'right_wrist_roll_joint',
            'left_hip_pitch_joint',
            'right_hip_pitch_joint',
            'left_wrist_pitch_joint',
            'right_wrist_pitch_joint',
            'left_knee_pitch_joint',
            'right_knee_pitch_joint',
            'left_ankle_pitch_joint',
            'right_ankle_pitch_joint',
            'left_ankle_roll_joint',
            'right_ankle_roll_joint',
            'L_index_proximal_joint',
            'L_middle_proximal_joint',
            'L_pinky_proximal_joint',
            'L_ring_proximal_joint',
            'L_thumb_proximal_yaw_joint',
            'R_index_proximal_joint',
            'R_middle_proximal_joint',
            'R_pinky_proximal_joint',
            'R_ring_proximal_joint',
            'R_thumb_proximal_yaw_joint',
            'L_index_intermediate_joint',
            'L_middle_intermediate_joint',
            'L_pinky_intermediate_joint',
            'L_ring_intermediate_joint',
            'L_thumb_proximal_pitch_joint',
            'R_index_intermediate_joint',
            'R_middle_intermediate_joint',
            'R_pinky_intermediate_joint',
            'R_ring_intermediate_joint',
            'R_thumb_proximal_pitch_joint',
            'L_thumb_distal_joint',
            'R_thumb_distal_joint',
        ]

        joint_names_gym = [
            'left_hip_roll_joint',
            'left_hip_yaw_joint',
            'left_hip_pitch_joint',
            'left_knee_pitch_joint',
            'left_ankle_pitch_joint',
            'left_ankle_roll_joint',
            'right_hip_roll_joint',
            'right_hip_yaw_joint',
            'right_hip_pitch_joint',
            'right_knee_pitch_joint',
            'right_ankle_pitch_joint',
            'right_ankle_roll_joint',
            'waist_yaw_joint',
            'waist_pitch_joint',
            'waist_roll_joint',
            'head_roll_joint',
            'head_pitch_joint',
            'head_yaw_joint',
            'left_shoulder_pitch_joint',
            'left_shoulder_roll_joint',
            'left_shoulder_yaw_joint',
            'left_elbow_pitch_joint',
            'left_wrist_yaw_joint',
            'left_wrist_roll_joint',
            'left_wrist_pitch_joint',
            'L_index_proximal_joint',
            'L_index_intermediate_joint',
            'L_middle_proximal_joint',
            'L_middle_intermediate_joint',
            'L_pinky_proximal_joint',
            'L_pinky_intermediate_joint',
            'L_ring_proximal_joint',
            'L_ring_intermediate_joint',
            'L_thumb_proximal_yaw_joint',
            'L_thumb_proximal_pitch_joint',
            'L_thumb_distal_joint',
            'right_shoulder_pitch_joint',
            'right_shoulder_roll_joint',
            'right_shoulder_yaw_joint',
            'right_elbow_pitch_joint',
            'right_wrist_yaw_joint',
            'right_wrist_roll_joint',
            'right_wrist_pitch_joint',
            'R_index_proximal_joint',
            'R_index_intermediate_joint',
            'R_middle_proximal_joint',
            'R_middle_intermediate_joint',
            'R_pinky_proximal_joint',
            'R_pinky_intermediate_joint',
            'R_ring_proximal_joint',
            'R_ring_intermediate_joint',
            'R_thumb_proximal_yaw_joint',
            'R_thumb_proximal_pitch_joint',
            'R_thumb_distal_joint',
        ]
        self.gym_adapter = gymutil.gym_adapter(joint_names_gym, joint_names_sim)

    def set_gains(self):
        """
        Set default stiffness (kps) and damping (kds) for joints.
        """
        self.get_articulation_controller().set_effort_modes('force')
        self.get_articulation_controller().switch_control_mode('effort')

        kps = self.gym_adapter.gym2sim(
            np.array(
                [
                    251.6250,
                    362.5200,
                    200.0000,
                    200.0000,
                    10.9805,
                    10.9805,
                    251.6250,
                    362.5200,
                    200.0000,
                    200.0000,
                    10.9805,
                    10.9805,
                    350.0000,
                    350.0000,
                    350.0000,
                    112.0600,
                    112.0600,
                    112.0600,
                    92.8500,
                    92.8500,
                    112.0600,
                    112.0600,
                    112.0600,
                    10.0000,
                    10.0000,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    92.8500,
                    92.8500,
                    112.0600,
                    112.0600,
                    112.0600,
                    10.0000,
                    10.0000,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                    5.0,
                ]
            )
        )
        kds = self.gym_adapter.gym2sim(
            np.array(
                [
                    14.7200,
                    10.0833,
                    11.0000,
                    11.0000,
                    0.6000,
                    0.6000,
                    14.7200,
                    10.0833,
                    11.0000,
                    11.0000,
                    0.6000,
                    0.6000,
                    15.0000,
                    15.0000,
                    15.0000,
                    3.1000,
                    3.1000,
                    3.1000,
                    2.5750,
                    2.5750,
                    3.1000,
                    3.1000,
                    3.1000,
                    1.0000,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    1.0000,
                    2.5750,
                    2.5750,
                    3.1000,
                    3.1000,
                    3.1000,
                    1.0000,
                    1.0000,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                    2.0,
                ]
            )
        )

        if kps is not None:
            kps = self._articulation_view._backend_utils.expand_dims(kps, 0)
        if kds is not None:
            kds = self._articulation_view._backend_utils.expand_dims(kds, 0)
        self._articulation_view.set_gains(kps=kps, kds=kds, save_to_usd=False)


@BaseRobot.register('GR1Robot')
class GR1Robot(BaseRobot):
    def __init__(self, config: RobotCfg, scene: Scene):
        super().__init__(config, scene)
        self._sensor_config = config.sensors
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'GR1 {config.name}: position    : ' + str(self._start_position))
        log.debug(f'GR1 {config.name}: orientation : ' + str(self._start_orientation))

        usd_path = config.usd_path

        log.debug(f'GR1 {config.name}: usd_path         : ' + str(usd_path))
        log.debug(f'GR1 {config.name}: config.prim_path : ' + str(config.prim_path))
        self.isaac_robot = GR1(
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

        self.isaac_robot.set_enabled_self_collisions(False)

    def _set_rigid_bodies(self):
        self._robot_base = self._rigid_body_map[self.config.prim_path + '/torso_link']
        self._imu_in_torso = self._rigid_body_map[self.config.prim_path + '/imu_link']

    def get_rigid_bodies(self) -> List[RigidPrim]:
        return self._rigid_body_map.values()

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
            'joint_positions': self.isaac_robot.get_joint_positions(),
            'joint_velocities': self.isaac_robot.get_joint_velocities(),
            'controllers': {},
            'sensors': {},
        }

        # common
        for c_obs_name, controller_obs in self.controllers.items():
            obs['controllers'][c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            obs['sensors'][sensor_name] = sensor_obs.get_data()
        return self._make_ordered(obs)
