from typing import Optional

from grutopia.core.config.metric import MetricUserConfig


class SimpleMetricCfg(MetricUserConfig):
    type: Optional[str] = 'SimpleMetric'
