from typing import List, Optional

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
        position (List[float], optional): The 3D coordinates of the object's position. Defaults to [0.0, 0.0, 0.0].
        scale (List[float], optional): Scaling factors along each axis. Defaults to [1.0, 1.0, 1.0].
        mass (float, optional): Mass of the object for physics simulation. Defaults to None.
        density (float, optional): Density of the object, used in conjunction with volume to calculate mass if mass is not provided. Defaults to None.
        collider (bool, optional): Determines if the object should participate in collision detection. Defaults to True.
        type (str): Specifies the type of the object, restricting to "UsdObj" or "DynamicCube". Mandatory and must be set correctly.
        usd_obj_param (UsdObj, optional): Parameters specific to "UsdObj" type objects. Defaults to None.
        color (List[float], optional): Color information for objects where applicable. Defaults to None.
        dynamic_cube_param (DynamicCube, optional): Parameters for "DynamicCube" type objects. Defaults to None.

    Raises:
        ValueError: If the 'type' attribute is set to a value other than "UsdObj" or "DynamicCube".

    Note:
        Ensure that the 'type' attribute is correctly set to either "UsdObj" or "DynamicCube" to avoid exceptions. The associated parameters (`usd_obj_param` and `dynamic_cube_param`) should be provided as needed based on the chosen object type.
    """

    # common
    name: str
    prim_path: str
    position: Optional[List[float]] = [0.0, 0.0, 0.0]
    orientation: Optional[List[float]] = [1.0, 0.0, 0.0, 0.0]
    scale: Optional[List[float]] = [1.0, 1.0, 1.0]

    type: str
