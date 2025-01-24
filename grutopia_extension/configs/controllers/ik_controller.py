from typing import Optional

from grutopia.core.config.robot import ControllerModel


class InverseKinematicsControllerCfg(ControllerModel):
    type: Optional[str] = 'InverseKinematicsController'
    robot_description_path: str
    robot_urdf_path: str
    end_effector_frame_name: str
    threshold: float
    reference: Optional[str] = None
