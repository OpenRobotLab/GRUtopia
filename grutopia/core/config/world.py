from typing import Optional

from pydantic import BaseModel


class World(BaseModel):
    physics_dt: Optional[float | str] = None
    rendering_dt: Optional[float | str] = None
