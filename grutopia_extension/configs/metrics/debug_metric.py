from typing import Optional

from grutopia.core.config.metric import MetricCfg


class DebugMetricCfg(MetricCfg):
    type: Optional[str] = 'DebugMetric'
