from typing import List, Optional, Tuple

from pydantic import BaseModel

from grutopia.core.config.robot.params import ControllerParams


class SensorModel(BaseModel):
    """Sensor config in robot_model config."""
    name: str
    prim_path: Optional[str] = None
    type: str

    # Fields from params.
    enable: Optional[bool] = True
    size: Optional[Tuple[int, int]] = None  # Camera only
    scan_rate: Optional[int] = None  # RPS. Lidar only


class ControllerModel(BaseModel, extra='allow'):
    """Controller config in robot_model config."""
    name: str
    type: str
    joint_names: Optional[List[str]] = None
    robot_description_path: Optional[str] = None
    robot_urdf_path: Optional[str] = None
    end_effector_frame_name: Optional[str] = None
    rmpflow_config_path: Optional[str] = None
    sub_controllers: Optional[List[ControllerParams]] = None
    forward_speed: Optional[float] = None
    rotation_speed: Optional[float] = None
    lateral_speed: Optional[float] = None
    threshold: Optional[float] = None
    policy_weights_path: Optional[str] = None
    recover_height: Optional[float] = None

    map_data_path: Optional[str] = None  # navi only, npy BOG (binary occupancy grid) file
    reference: Optional[str] = None  # ik only, world/robot/arm_base, default to world

    # Planner controller
    planner: Optional[str] = None  # for planning policy.
    model: Optional[str] = None  # for planning policy model
    model_path: Optional[str] = None  # for planning policy, weight path of model


class RobotModel(BaseModel):
    """Robot config in robot_model config."""
    type: str
    usd_path: Optional[str] = None  # If Optional, use default usd_path
    controllers: Optional[List[ControllerModel]] = None
    sensors: Optional[List[SensorModel]] = None
    gains: Optional[List[float]] = None


class RobotModels(BaseModel):
    """robot_model config file structure."""
    robots: Optional[List[RobotModel]] = None
