from typing import Optional

from grutopia.core.config import RobotCfg
from grutopia.macros import gm
from grutopia_extension.configs.controllers import (
    FrankaMocapTeleopControllerCfg,
    GripperControllerCfg,
    RMPFlowControllerCfg,
)
from grutopia_extension.configs.sensors import MocapControlledCameraCfg

rmpflow_controller_cfg = RMPFlowControllerCfg(
    name='rmpflow_controller',
    robot_description_path=gm.ASSET_PATH + '/robots/franka/rmpflow/robot_descriptor.yaml',
    robot_urdf_path=gm.ASSET_PATH + '/robots/franka/lula_franka_gen.urdf',
    rmpflow_config_path=gm.ASSET_PATH + '/robots/franka/franka_rmpflow_common.yaml',
    end_effector_frame_name='right_gripper',
)

gripper_controller_cfg = GripperControllerCfg(
    name='gripper_controller',
)

teleop_cfg = FrankaMocapTeleopControllerCfg(
    name='franka_mocap_teleop_controller',
    scale=(0.25, 0.75, 0.75),
    target_position=(0.0, 0.0, 1.0),
    origin_xyz=(0.46, 0, 0.38),
    origin_xyz_angle=(0, 0, 0),
    sub_controllers=[rmpflow_controller_cfg, gripper_controller_cfg],
)

lh_controlled_camera_cfg = MocapControlledCameraCfg(
    name='lh_controlled_camera',
    prim_path='logo_link/lh_controlled_camera',
    translation=(0.0, 0.0, 5.0),
    orientation=(0.70710678, 0.0, 0.70710678, 0.0),
)


class MocapControlledFrankaRobotCfg(RobotCfg):
    # meta info
    name: Optional[str] = 'mocap_controlled_franka'
    type: Optional[str] = 'MocapControlledFrankaRobot'
    prim_path: Optional[str] = '/mocap_controlled_franka'
    create_robot: Optional[bool] = True
    usd_path: Optional[str] = gm.ASSET_PATH + '/robots/franka/mocap_teleop_franka.usd'
