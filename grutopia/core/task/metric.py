from abc import ABC, abstractmethod
from functools import wraps

from grutopia.core.config.metric import MetricUserConfig
from grutopia.core.runtime.task_runtime import TaskRuntime


class BaseMetric(ABC):
    metrics = {}

    def __init__(self, config: MetricUserConfig, task_runtime: TaskRuntime):
        self.config = config
        self.name = config.name
        self.task_runtime = task_runtime
        self.metric_config = config.metric_config

    @abstractmethod
    def reset(self):
        raise NotImplementedError(f'`reset` function of {self.name} is not implemented')

    @abstractmethod
    def update(self, *args):
        """
        This function is called at each world step.
        """
        raise NotImplementedError(f'`update` function of {self.name} is not implemented')

    @abstractmethod
    def calc(self):
        """
        This function is called to calculate the metrics when the episode is terminated.
        """
        raise NotImplementedError(f'`calc` function of {self.name} is not implemented')

    @classmethod
    def register(cls, name: str):
        """
        This function is used to register a metric class.(decorator)
        Args:
            name(str): name of the metric
        """

        def decorator(metric_class):
            cls.metrics[name] = metric_class

            @wraps(metric_class)
            def wrapped_function(*args, **kwargs):
                return metric_class(*args, **kwargs)

            return wrapped_function

        return decorator


def create_metric(config: MetricUserConfig, task_runtime: TaskRuntime):
    if config.type not in BaseMetric.metrics:
        raise KeyError(
            f"""The metric {config.type} is not registered, please register it using `@BaseMetric.register`""")
    metric_cls = BaseMetric.metrics[config.type]
    return metric_cls(config, task_runtime)
