from typing import Optional

from grutopia.core.config.metric import MetricUserConfig


class DebugMetricCfg(MetricUserConfig):
    type: Optional[str] = 'DebugMetric'
