from typing import Optional

from grutopia.core.config.metric import MetricUserConfig


class RecordingMetricCfg(MetricUserConfig):
    type: Optional[str] = 'RecordingMetric'
    robot_name: str
    fields: list = None  # fields that need to be recorded.
