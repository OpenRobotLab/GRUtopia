from typing import Optional

from grutopia.core.config.metric import MetricCfg


class SimpleMetricCfg(MetricCfg):
    type: Optional[str] = 'SimpleMetric'
