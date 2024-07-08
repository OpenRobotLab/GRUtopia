from typing import Optional

from pydantic import BaseModel


class MetricUserConfig(BaseModel):
    """
    MetricUserConfig
    """
    type: str
    name: Optional[str]
