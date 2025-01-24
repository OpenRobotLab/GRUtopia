from typing import Optional

from grutopia.core.config.metric import MetricUserConfig


class ResetTimeMetricCfg(MetricUserConfig):
    type: Optional[str] = 'ResetTimeMetric'
