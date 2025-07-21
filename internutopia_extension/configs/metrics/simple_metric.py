from typing import Optional

from internutopia.core.config.metric import MetricCfg


class SimpleMetricCfg(MetricCfg):
    type: Optional[str] = 'SimpleMetric'
    robot_name: str
