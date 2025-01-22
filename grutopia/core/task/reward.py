from abc import ABC, abstractmethod
from functools import wraps
from typing import Any, Dict

from grutopia.core.config.task import RewardConfig
from grutopia.core.task import BaseTask


class BaseReward(ABC):
    rewards = {}

    def __init__(self, task: BaseTask, settings: Dict[str, Any]):
        self.state = None
        self.task = task
        self.settings = settings
        self.init_setting()

    def init_setting(self):
        pass

    @abstractmethod
    def reset(self):
        self.state = None

    @abstractmethod
    def calc(self) -> float:
        raise NotImplementedError(f'`calc` function of {self.name} is not implemented')

    @abstractmethod
    def _calc_next_state(self):
        raise NotImplementedError(f'`_calc_next_state` function of {self.name} is not implemented')

    @classmethod
    def register(cls, name: str):
        """
        This function is used to register a reward class.(decorator)
        Args:
            name(str): name of the reward
        """

        def decorator(reward_class):
            cls.rewards[name] = reward_class

            @wraps(reward_class)
            def wrapped_function(*args, **kwargs):
                return reward_class(*args, **kwargs)

            return wrapped_function

        return decorator


def create_reward(reward_config: RewardConfig, task: BaseTask):
    if reward_config.reward_type not in BaseReward.rewards:
        raise KeyError(
            f"""The reward {reward_config.reward_type} is not registered, please register it using `@BaseReward.register`"""
        )
    reward_cls = BaseReward.rewards[reward_config.reward_type]
    return reward_cls(task, reward_config.reward_settings)
