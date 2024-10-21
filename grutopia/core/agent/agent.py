import asyncio
from abc import ABC, abstractmethod
from functools import wraps
from threading import Thread
from typing import Dict

from grutopia.core.config.agent import AgentConfig
from grutopia.core.datahub import DataHub
from grutopia.core.util import log
from grutopia.core.util.chat import AgentChat


def start_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()


class BaseAgent(ABC):
    """Base class for all agents."""
    agents = {}
    # async status
    loop: Thread | None = None
    agent_step_loop = asyncio.new_event_loop()

    def __init__(self, task_name: str, robot_name: str | None, agent_config: Dict, sync_mode: str, extra: Dict):
        self.task_name: str = task_name
        self.robot_name: str = robot_name
        self.sync_mode: str = sync_mode
        self.agent_config: Dict = agent_config
        self._step_over: bool = True
        self.extra: Dict = extra
        self.chat: AgentChat = AgentChat(self.task_name, self.robot_name)

    def get_observation(self) -> Dict:
        """
        Get observation from Datahub.
        Called in self.decision_making by users.

        Returns:
            Dict: observation dict.
        """
        if self.robot_name:
            return DataHub.get_obs_by_task_name_and_robot_name(self.task_name, self.robot_name)
        else:
            return DataHub.get_obs_by_task_name(self.task_name)

    @staticmethod
    def set_actions(action_dict: Dict):
        """
        Set actions for this task (for all robots) to datahub.
        Called in self.decision_making by users.

        Args:
            action_dict (Dict): Dict with actions
        """
        DataHub.set_actions(action_dict)

    def terminate(self):
        """
        Set task finish for task with self.task_name to Datahub.
        Called in self.decision_making by users.
        """
        log.info(f'Task {self.task_name} finished')
        DataHub.set_episode_finished(self.task_name)

    @abstractmethod
    def decision_making(self):
        """
        Make decision with `self.observation` and set action to `self.`
        """
        raise NotImplementedError

    def step(self):
        """
        Get observation, make decision, set actions.
        """
        if self.sync_mode == 'sync':
            self.decision_making()
        elif self.sync_mode == 'async':
            self._step_async()

    async def _step(self):
        """
        Get observation, make decision, set action. Async.
        """
        self.decision_making()
        self._step_over = True

    def _step_async(self):
        """
        Wrap `_step` into a coroutine.
        """
        if self._step_over:
            self._step_over = False
            log.debug(f'============= submit coroutine {self.task_name} ===========')
            asyncio.run_coroutine_threadsafe(self._step(), loop=self.agent_step_loop)

    @classmethod
    def agent_loop_start(cls):
        """
        Start agent asyncio event loop in another thread.
        """
        if cls.loop is None:
            cls.loop = Thread(target=start_loop, args=(cls.agent_step_loop, ))
            cls.loop.daemon = True
            cls.loop.start()

    @classmethod
    def register(cls, name: str):
        """Register an agent class with its name(decorator).

        Args:
            name(str): name of the agent class.
        """

        def decorator(robot_class):
            cls.agents[name] = robot_class

            @wraps(robot_class)
            def wrapped_function(*args, **kwargs):
                return robot_class(*args, **kwargs)

            return wrapped_function

        return decorator


def create_agent(config: AgentConfig, task_name: str, extra: Dict):
    agent_inst: BaseAgent = BaseAgent.agents[config.type](task_name=task_name,
                                                          robot_name=config.robot_name,
                                                          sync_mode=config.sync_mode,
                                                          agent_config=config.agent_config,
                                                          extra=extra)
    return agent_inst
