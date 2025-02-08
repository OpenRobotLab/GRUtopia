from typing import Optional, Tuple

from pydantic import BaseModel

from grutopia.core.config.scene.object_params import DynamicCube, UsdObj


class Scene(BaseModel):
    """
    A class representing a scene, inheriting from BaseModel.

    This class provides a structure for storing information related to a scene in a project.
    It includes basic attributes like type, name, and path, which can be used to categorize and locate scenes within a larger context.

    Attributes:
        type (str): The type of the scene, indicating its category or nature.
        name (Optional[str]): The name of the scene, which can be used for identification. Defaults to None.
        path (Optional[str]): The file system path to the scene, enabling direct access. Defaults to None.
    """

    type: str
    name: Optional[str] = None
    path: Optional[str] = None


class ObjectCfg(BaseModel):
    """
    Represents a customizable object within a scene, with support for physics and different object types.

    This class defines an object that can be placed within a virtual environment, providing essential properties for both graphical representation and physical simulation. It supports varying object types with specific parameters, ensuring flexibility and customizability for different use cases.

    Attributes:
        name (str): The unique identifier for the object.
        prim_path (str): The path indicating the object's position within the scene hierarchy.
        position (Tuple[float, float, float], optional): The 3D coordinates of the object's position. Defaults to (0.0, 0.0, 0.0).
        orientation (Tuple[float, float, float, float], optional): The quaternion representation of the object's orientation. Defaults to (1.0, 0.0, 0.0, 0.0).
        scale (Tuple[float, float, float], optional): Scaling factors along each axis. Defaults to (1.0, 1.0, 1.0).
        type (str): Specifies the type of the object.
    """

    # common
    name: str
    prim_path: str
    position: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    orientation: Optional[Tuple[float, float, float, float]] = (1.0, 0.0, 0.0, 0.0)
    scale: Optional[Tuple[float, float, float]] = (1.0, 1.0, 1.0)

    type: str
