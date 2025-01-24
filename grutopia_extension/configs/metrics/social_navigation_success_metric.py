from typing import Optional

from pydantic import BaseModel

from grutopia.core.config.metric import MetricUserConfig


class SocialNavigationSuccessMetricConfig(BaseModel):
    navigation_error_threshold: int


class SocialNavigationSuccessMetricCfg(MetricUserConfig):
    type: Optional[str] = 'SocialNavigationSuccessMetric'
    metric_config: SocialNavigationSuccessMetricConfig
