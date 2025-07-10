from typing import List, Optional, Tuple

from pydantic import BaseModel

from grutopia.core.config.sensor import SensorCfg


class BaseCfg(BaseModel):
    def update(self, **kwargs):
        return self.model_copy(update=kwargs, deep=True)


class ControllerCfg(BaseCfg, extra='allow'):
    """
    A specialized model representing controllers within a system, inheriting from BaseModel with an extended configuration to allow additional keys.

    This class serves as a data structure to encapsulate controller-related information crucial for configuring and managing control systems, particularly in robotic applications. It provides a structured way to define controllers, including their name, type, sub-controllers (if any), and a reference frame.

    Attributes:
        name (str): The unique identifier for the controller.
        type (str): Specifies the controller's type or category, determining its behavior and functionality.
        sub_controllers (Optional[List[ControllerModel]], optional): A list of nested 'ControllerModel' instances, enabling hierarchical controller structures. Defaults to None.

    Usage:
        Instantiate a `ControllerModel` to define a controller configuration. Optionally, nest other `ControllerModel` instances within the `sub_controllers` attribute to model complex controller hierarchies.

    Example Type Hints Usage:
        - `name`: Always a string.
        - `type`: String defining controller type.
        - `sub_controllers`: A list of `ControllerModel` instances or None.
    """

    name: str
    type: str
    sub_controllers: Optional[List['ControllerCfg']] = None
    # reference: Optional[str] = None  # ik only, world/robot/arm_base, default to world

    # joint_names: Optional[List[str]] = None
    # robot_description_path: Optional[str] = None
    # robot_urdf_path: Optional[str] = None
    # end_effector_frame_name: Optional[str] = None
    # rmpflow_config_path: Optional[str] = None
    # sub_controllers: Optional[List[ControllerParams]] = None
    # forward_speed: Optional[float] = None
    # rotation_speed: Optional[float] = None
    # lateral_speed: Optional[float] = None
    # threshold: Optional[float] = None
    # policy_weights_path: Optional[str] = None
    # recover_height: Optional[float] = None
    #
    # map_data_path: Optional[str] = None  # navi only, npy BOG (binary occupancy grid) file
    # reference: Optional[str] = None  # ik only, world/robot/arm_base, default to world
    #
    # # Planner controller
    # planner: Optional[str] = None  # for planning policy.
    # model: Optional[str] = None  # for planning policy model
    # model_path: Optional[str] = None  # for planning policy, weight path of model


class RobotCfg(BaseCfg):
    """
    Represents a robot configuration with customizable attributes and optional components like controllers and sensors.

    This RobotCfg class is designed to store metadata and common configurations for robotic models. It inherits from BaseCfg,
    providing a structured way to define a robot's properties within a simulation or robotic application context. The model includes
    details about the robot's USD (Universal Scene Description) path, initial position, orientation, and other settings crucial for
    simulation initialization and control.

    Attributes:
        name (str): The name identifier for the robot.
        type (str): The type or category of the robot.
        prim_path (str): The USD prim path where the robot is located or should be instantiated within a scene.
        create_robot (bool, optional): Flag indicating whether to create the robot instance during simulation setup. Defaults to True.
        usd_path (Optional[str], optional): The file path to the USD containing the robot definition. If None, a default path is used.

        position (Optional[List[float]], optional): Initial position of the robot in world frame. Defaults to (0.0, 0.0, 0.0).
        orientation (Optional[List[float]], optional): Initial orientation of the robot in quaternion. Defaults to None.
        scale (Optional[List[float]], optional): Scaling factor for the robot. Defaults to None.

        controllers (Optional[List[ControllerCfg]], optional): List of controller configurations attached to the robot. Defaults to None.
        sensors (Optional[List[SensorCfg]], optional): List of sensor configurations attached to the robot. Defaults to None.
    """

    # meta info
    name: str
    type: str
    prim_path: str
    create_robot: bool = True
    usd_path: Optional[str] = None  # If Optional, use default usd_path

    # common config
    position: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    orientation: Optional[Tuple[float, float, float, float]] = None
    scale: Optional[Tuple[float, float, float]] = None

    controllers: Optional[List[ControllerCfg]] = None
    sensors: Optional[List[SensorCfg]] = None
