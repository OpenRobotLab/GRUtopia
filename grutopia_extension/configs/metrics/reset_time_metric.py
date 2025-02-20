from typing import Optional

from grutopia.core.config.metric import MetricCfg


class ResetTimeMetricCfg(MetricCfg):
    type: Optional[str] = 'ResetTimeMetric'
