from typing import List, Optional

from pydantic import BaseModel

from grutopia.core.config.scene.object_params import DynamicCube, UsdObj


class Scene(BaseModel):
    type: str
    name: Optional[str] = None
    path: Optional[str] = None


class Object(BaseModel):
    # common
    name: str
    prim_path: str
    position: Optional[List[float]] = [0.0, 0.0, 0.0]
    scale: Optional[List[float]] = [1.0, 1.0, 1.0]

    # physics
    mass: Optional[float] = None
    density: Optional[float] = None
    collider: Optional[bool] = True

    # Set type in ["UsdObj", "DynamicCube"]
    # If not, raise error
    type: str

    # params for each type of
    usd_obj_param: Optional[UsdObj] = None
    color: Optional[List[float]] = None
    dynamic_cube_param: Optional[DynamicCube] = None
