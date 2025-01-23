from typing import List, Optional

from grutopia.core.config import RobotModel
from grutopia.core.config.robot import ControllerModel, SensorModel
from grutopia_extension.config.controllers import HumanoidMoveBySpeedControllerCfg
from grutopia_extension.config.sensors import RepCameraCfg

humanoid_move_by_speed_controller = HumanoidMoveBySpeedControllerCfg(
    name='move_by_speed',
    type='HumanoidMoveBySpeedController',
    policy_weights_path='GRUtopia/assets/policy/weights/h1_loco_model_20000.pt',
    joint_names=[
        'left_hip_yaw_joint',
        'right_hip_yaw_joint',
        'torso_joint',
        'left_hip_roll_joint',
        'right_hip_roll_joint',
        'left_shoulder_pitch_joint',
        'right_shoulder_pitch_joint',
        'left_hip_pitch_joint',
        'right_hip_pitch_joint',
        'left_shoulder_roll_joint',
        'right_shoulder_roll_joint',
        'left_knee_joint',
        'right_knee_joint',
        'left_shoulder_yaw_joint',
        'right_shoulder_yaw_joint',
        'left_ankle_joint',
        'right_ankle_joint',
        'left_elbow_joint',
        'right_elbow_joint',
    ],
)

humanoid_camera = RepCameraCfg(name='camera', type='RepCamera', prim_path='logo_link/Camera', size=(640, 480))

humanoid_tp_camera = RepCameraCfg(name='tp_camera', type='RepCamera', prim_path='torso_link/TPCamera', size=(640, 480))


class HumanoidRobot(RobotModel):
    # meta info
    name: Optional[str] = 'h1'
    type: Optional[str] = 'HumanoidRobot'
    prim_path: Optional[str] = '/World/h1'
    create_robot: Optional[bool] = True
    usd_path: Optional[str] = 'GRUtopia/assets/robots/h1/h1.usd'

    # common config
    position: Optional[List[float]] = (5.58, -0.77, 1.05)
    orientation: Optional[List[float]] = None
    scale: Optional[List[float]] = (1, 1, 1)

    controllers: Optional[List[ControllerModel]] = ()
    sensors: Optional[List[SensorModel]] = ()
    gains: Optional[List[float]] = (0.0, 0.0)
