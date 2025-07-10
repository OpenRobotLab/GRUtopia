from typing import Optional, Tuple

from pydantic import BaseModel


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

    name: str
    prim_path: str
    position: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)
    orientation: Optional[Tuple[float, float, float, float]] = (1.0, 0.0, 0.0, 0.0)
    scale: Optional[Tuple[float, float, float]] = (1.0, 1.0, 1.0)
    type: str


class DynamicCubeCfg(ObjectCfg):
    """
    Configuration for a dynamic cube object in the scene.

    This class specifies the configuration parameters for a dynamic cube, allowing customization of its physical and visual properties.

    Attributes:
        color (Optional[Tuple[float, float, float]], optional): RGB color of the cube. Defaults to None.
        mass (Optional[float], optional): Mass of the cube in kilograms. Defaults to None.
    """

    type: Optional[str] = 'DynamicCube'
    color: Optional[Tuple[float, float, float]] = None
    mass: Optional[float] = None


class UsdObjCfg(ObjectCfg):
    """
    Configuration for a USD object in the scene.

    This class specifies the configuration parameters for a USD object.

    Attributes:
        usd_path (str): Path to the USD file.
        collider (bool, optional): Whether the object enables collider.
    """

    type: Optional[str] = 'UsdObject'
    usd_path: str
    collider: Optional[bool] = True
