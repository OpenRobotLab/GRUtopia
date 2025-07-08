from typing import Any, OrderedDict

import gymnasium as gym

from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util import log


class Env(gym.Env):
    """
    Gym Env for a single environment with a single learning agent.
    """

    RESET_INFO_TASK_RUNTIME = 'task_runtime'

    def __init__(self, simulator_runtime: SimulatorRuntime) -> None:
        self._render = None
        self._runtime = simulator_runtime
        self._robot_name = None
        self._current_task_name = None
        self._validate()

        from grutopia.core.runner import SimulatorRunner  # noqa E402.

        self._runner = SimulatorRunner(simulator_runtime=simulator_runtime)

        # ========================= import space ============================
        import grutopia.core.util.space as space  # noqa E402.

        self._space = space
        self.action_space = self._get_action_space()
        self.observation_space = self._get_observation_space()
        # ===================================================================

        log.info(f'==================== {self._robot_name} ======================')
        return

    def _validate(self):
        """This method is designed for **only** 1 env + 1 robot."""
        if self._runtime.env_num > 1:
            raise ValueError(f'Only support single env now, but env num is {self._runtime.env_num}')

        episodes = self._runtime.task_runtime_manager.episodes
        log.debug(f'================ len(episodes): {len(episodes)} ==================')

        _episode_sample = self._runtime.task_runtime_manager.episodes[0]

        if len(_episode_sample.robots) == 0:
            return
        if len(_episode_sample.robots) != 1:
            raise ValueError(
                f'Only support single agent now, but episode requires {len(_episode_sample.robots)} agents'
            )
        self._robot_name = f'{_episode_sample.robots[0].name}'

    def _get_action_space(self) -> gym.Space:
        print(self._runtime)
        return self._space.get_action_space_by_task(self._runtime.config['task_config']['type'])

    def _get_observation_space(self) -> gym.Space:
        return self._space.get_observation_space_by_task(self._runtime.config['task_config']['type'])

    def reset(self, *, seed=None, options=None) -> tuple[OrderedDict | None, dict | None]:
        """Resets the environment to an initial internal state, returning an initial observation and info.

        Args:
            seed (optional int): The seed that is used to initialize the environment's PRNG (`np_random`).
            options (optional dict): Additional information to specify how the environment is reset (optional,
                depending on the specific environment)

        Returns:
            observation (OrderedDict): Observation of the initial state.
            info (dict): Contains the key `task_runtime` if there is an unfinished task
        """
        info = {}
        obs = OrderedDict()

        origin_obs, task_runtime = self.runner.reset(None if self._current_task_name is None else [0])
        if None in task_runtime:
            log.info('All episodes have finished.')
            return None, None

        self._current_task_name = task_runtime[0].name
        info[Env.RESET_INFO_TASK_RUNTIME] = task_runtime[0]
        if self._robot_name:
            obs = origin_obs[0][self._robot_name]

        return obs, info

    def warm_up(self, steps: int = 10, render: bool = True, physics: bool = True):
        """
        Warm up the env by running a specified number of steps.

        Args:
            steps (int): The number of warm-up steps to perform. Defaults to 10.
            render (bool): Whether to render the scene during warm-up. Defaults to True.
            physics (bool): Whether to enable physics during warm-up. Defaults to True.
        """
        self.runner.warm_up(steps, render, physics)

    def step(self, action: Any) -> tuple[Any, float, bool, bool, dict[str, Any] | None]:
        """
        run step with given action(with isaac step)

        TODO: Implement the conversion between dict and action space/obs space

        Args:
            action (Any): an action provided by the agent to update the environment state.

        Returns:
            observation (Any): An element of the environment's :attr:`observation_space` as the next observation due to the agent actions.
            reward (float): The reward as a result of taking the action.
            terminated (bool): Whether the agent reaches the terminal state. If true, the user needs to call :meth:`reset`.
            truncated (bool): Whether the truncation condition outside the scope of the MDP is satisfied.
                Typically, this is a timelimit, but could also be used to indicate an agent physically going out of bounds.
                Can be used to end the episode prematurely before a terminal state is reached.
                If true, the user needs to call :meth:`reset`.
            info (dict): Contains auxiliary diagnostic information (helpful for debugging, learning, and logging).
                Currently, it contains nothing.
        """

        obs = {}
        reward = 0.0
        terminated = False
        truncated = False
        info = None

        if self._current_task_name is None:
            return obs, reward, terminated, truncated, info

        _actions = [{self._robot_name: action}]
        origin_obs, terminated_status, rewards = self._runner.step(_actions)

        if rewards[0] != -1:
            reward = rewards[0]

        if self._robot_name:
            obs = origin_obs[0][self._robot_name]
        terminated = terminated_status[0]

        return obs, reward, terminated, truncated, info

    @property
    def runner(self):
        return self._runner

    @property
    def is_render(self):
        return self._render

    @property
    def active_runtimes(self):
        return self._runtime.active_runtime()

    def get_dt(self):
        """
        Get dt of simulation environment.
        Returns:
            dt.
        """
        return self._runner.dt

    def get_observations(self) -> dict[Any, Any] | Any:
        """
        Get observations from Isaac environment

        Returns:
            observation (gym.Space): observation
        """
        if self._current_task_name is None:
            return {}

        _obs = self._runner.get_obs()
        if self._robot_name is None:
            return {}
        return _obs[0][self._robot_name]

    def render(self, mode='human'):
        pass

    def close(self):
        """close the environment"""
        self._runtime.simulation_app.close()
        return

    @property
    def simulation_runtime(self):
        """config of simulation environment"""
        return self._runtime.active_runtime()

    @property
    def simulation_app(self):
        """simulation app instance"""
        return self._runtime.simulation_app

    def finished(self) -> bool:
        """check if all tasks are finished"""
        return len(self._runner.current_tasks) == 0
