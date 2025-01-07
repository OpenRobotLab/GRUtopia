import asyncio
from abc import ABC, abstractmethod
from functools import wraps
from threading import Thread
from typing import Any, Dict, Optional

from grutopia.core.config.agent import AgentConfig
from grutopia.core.datahub import DataHub
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.util import log
from grutopia.core.util.chat import AgentChat
from grutopia.core.util.space import get_action_space_by_task, get_observation_space_by_task


def start_loop(loop):
    asyncio.set_event_loop(loop)
    loop.run_forever()


class BaseAgent(ABC):
    """Base class for all agents."""

    agents = {}
    # async status
    loop: Thread | None = None
    agent_step_loop = asyncio.new_event_loop()

    def __init__(self, task_name: str, robot_name: str | None, agent_config: Dict, sync_mode: str,
                 task_runtime: TaskRuntime):
        self.task_name: str = task_name
        self.robot_name: str = robot_name + '_' + str(task_runtime.env.env_id)
        self.sync_mode: str = sync_mode
        self.agent_config: Dict = agent_config
        self._step_over: bool = True
        self.task_runtime: TaskRuntime = task_runtime
        self.chat: AgentChat = AgentChat(self.task_name, self.robot_name)
        self.action_space = get_action_space_by_task(self.task_runtime.type)
        self.observation_space = get_observation_space_by_task(self.task_runtime.type)

    def _get_observation(self) -> Any:
        """
        Get observation from Datahub.
        Called in self.decision_making by users.
        This method only applies to async mode.

        Returns:
            Any: observation data.
        """

        # TODO: make datahub more generic
        if self.robot_name:
            return DataHub.get_obs_by_task_name_and_robot_name(self.task_name, self.robot_name)
        else:
            return DataHub.get_obs_by_task_name(self.task_name)

    def set_observation(self, obs: Dict[str, Dict[str, Any]]) -> Dict:
        """
        Set observation in Datahub.
        Called in self.decision_making by users.

        Returns:
            Dict: observation dict.
        """
        if self.robot_name:
            return DataHub.set_obs_by_task_name_and_robot_name(self.task_name, self.robot_name, obs=obs)
        else:
            return DataHub.set_obs_by_task_name(self.task_name, obs)

    @staticmethod
    def _set_actions(actions: Any):
        """
        Set actions for this task (for all robots) to datahub.
        Called in self.decision_making by users.
        This method only applies to async mode.

        Args:
            action (Any): action data
        """
        # TODO: make datahub more generic
        DataHub.set_actions(actions)

    def terminate(self) -> Any:
        """
        Set task finish by sending termination action.
        Called in self.decision_making by users.

        Returns:
        Any: The termination action data.
        """
        raise NotImplementedError

    @abstractmethod
    def decision_making(self, obs: Any) -> Any:
        """
        Make decision with the given observation and return action

        Args:
            obs (Any): The observation data.

        Returns:
            Any: The action data.
        """
        raise NotImplementedError

    def step(self, obs: Optional[Any] = None) -> Optional[Any]:
        """
        Get observation, make decision, set actions.
        For sync mode, the action will be returned directly.
        For async mode, the action will be set to datahub.

        Args:
            obs (Optional[Any]): The observation data used for decision making.

        Returns:
            Optional[Any]: The action to be taken.
        """
        if self.sync_mode == 'sync':
            return self.decision_making(obs)
        elif self.sync_mode == 'async':
            self._step_async()

    async def _step(self):
        """
        Get observation, make decision, set action. Async.
        """
        obs = self._get_observation()
        self._set_actions(self.decision_making(obs))
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


def create_agent(config: AgentConfig, task_name: str, task_runtime: TaskRuntime):
    agent_inst: BaseAgent = BaseAgent.agents[config.type](
        task_name=task_name,
        robot_name=config.robot_name,
        sync_mode=config.sync_mode,
        agent_config=config.agent_config,
        task_runtime=task_runtime,
    )
    return agent_inst
