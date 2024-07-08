from typing import List, Optional

from pydantic import BaseModel


class DynamicCube(BaseModel):
    color: Optional[List[float]] = None
