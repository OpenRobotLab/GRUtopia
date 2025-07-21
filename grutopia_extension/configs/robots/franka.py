from typing import Optional

from grutopia.core.config import RobotCfg
from grutopia.macros import gm
from grutopia_extension.configs.controllers import (
    GripperControllerCfg,
    InverseKinematicsControllerCfg,
)

arm_ik_cfg = InverseKinematicsControllerCfg(
    name='arm_ik_controller',
    robot_description_path=gm.ASSET_PATH + '/robots/franka/rmpflow/robot_descriptor.yaml',
    robot_urdf_path=gm.ASSET_PATH + '/robots/franka/lula_franka_gen.urdf',
    end_effector_frame_name='right_gripper',
    threshold=0.01,
)

gripper_cfg = GripperControllerCfg(
    name='gripper_controller',
)


class FrankaRobotCfg(RobotCfg):
    # meta info
    name: Optional[str] = 'franka'
    type: Optional[str] = 'FrankaRobot'
    prim_path: Optional[str] = '/franka'
    usd_path: Optional[str] = gm.ASSET_PATH + '/robots/franka/franka.usd'
