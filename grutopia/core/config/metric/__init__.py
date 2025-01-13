from typing import Dict, Optional

from pydantic import BaseModel


class MetricUserConfig(BaseModel):
    """
    MetricUserConfig
    """

    type: str
    name: Optional[str] = None
    metric_config: Optional[Dict] = {}
