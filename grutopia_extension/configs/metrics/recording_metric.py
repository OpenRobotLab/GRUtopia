from typing import Optional

from grutopia.core.config.metric import MetricCfg


class RecordingMetricCfg(MetricCfg):
    type: Optional[str] = 'RecordingMetric'
    robot_name: str
    fields: list = None  # fields that need to be recorded.
