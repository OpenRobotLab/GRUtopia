from typing import Dict, Optional, Union

from pydantic import BaseModel


class MetricCfg(BaseModel):
    """
    A configuration model for users to define metrics.

    This class represents a user-defined configuration for metrics, which includes
    the type of metric, an optional name for the metric, and an optional dictionary
    of additional metric configurations. It is designed to be used in conjunction
    with systems that evaluate or monitor performance based on specified metrics.

    Attributes:
        type (str): The type of the metric, specifying how the metric is calculated or what it measures.
        name (Optional[str]): An optional name for the metric, allowing users to provide a descriptive label.
        metric_config (Optional[Dict]): An optional dictionary containing specific configuration parameters
            for the metric, which may vary depending on the metric type.

    Example Usage:
    ```python
    config = MetricCfg(
        type="accuracy",
        name="Training Accuracy",
        metric_config={"threshold": 0.9}
    )
    ```
    """

    type: str
    name: Optional[str] = None
    metric_config: Optional[Union[Dict, BaseModel]] = {}
