from typing import Optional

from pydantic import BaseModel

from grutopia.core.config.metric import MetricCfg


class SocialNavigationSuccessMetricConfig(BaseModel):
    navigation_error_threshold: int


class SocialNavigationSuccessMetricCfg(MetricCfg):
    type: Optional[str] = 'SocialNavigationSuccessMetric'
    metric_config: SocialNavigationSuccessMetricConfig
