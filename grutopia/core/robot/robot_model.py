from typing import List, Optional, Tuple

from pydantic import BaseModel

from grutopia.core.config.robot.params import ControllerParams


class SensorModel(BaseModel):
    """Sensor config in robot_model config."""
    name: str
    prim_path: Optional[str]
    type: str

    # Fields from params.
    enable: Optional[bool] = True
    size: Optional[Tuple[int, int]]  # Camera only
    scan_rate: Optional[int]  # RPS. Lidar only


class ControllerModel(BaseModel):
    """Controller config in robot_model config."""
    name: str
    type: str
    joint_names: Optional[List[str]]
    robot_description_path: Optional[str]
    robot_urdf_path: Optional[str]
    end_effector_frame_name: Optional[str]
    rmpflow_config_path: Optional[str]
    sub_controllers: Optional[List[ControllerParams]]
    forward_speed: Optional[float]
    rotation_speed: Optional[float]
    lateral_speed: Optional[float]
    threshold: Optional[float]
    policy_weights_path: Optional[str]

    map_data_path: Optional[str]  # navi only, npy BOG (binary occupancy grid) file
    reference: Optional[str]  # ik only, world/robot/arm_base, default to world

    # Planner controller
    planner: Optional[str]  # for planning policy.
    model: Optional[str]  # for planning policy model
    model_path: Optional[str]  # for planning policy, weight path of model


class RobotModel(BaseModel):
    """Robot config in robot_model config."""
    type: str
    usd_path: Optional[str]  # If Optional, use default usd_path
    controllers: Optional[List[ControllerModel]]
    sensors: Optional[List[SensorModel]]
    gains: Optional[List[float]]
    joint_names: Optional[List[str]]


class RobotModels(BaseModel):
    """robot_model config file structure."""
    robots: Optional[List[RobotModel]]
