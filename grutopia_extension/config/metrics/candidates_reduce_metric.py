from typing import Optional

from grutopia.core.config.metric import MetricUserConfig


class ECRMetricCfg(MetricUserConfig):
    type: Optional[str] = 'ECRMetric'
