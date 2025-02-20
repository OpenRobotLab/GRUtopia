from typing import Optional

from grutopia.core.config.metric import MetricCfg


class ECRMetricCfg(MetricCfg):
    type: Optional[str] = 'ECRMetric'
