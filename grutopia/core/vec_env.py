from typing import Any, Dict, List, OrderedDict, Tuple, Union

from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util import log


class Env:
    """
    Represents an environment for multi env and multi agents. **Not gymnasium compatible**.

    This class encapsulates the capability to reset, step, and close environments
    within a simulation framework. It provides properties to access the runner, active
    runtimes, simulation configuration, and other relevant components. The class is
    designed to manage the lifecycle of simulation tasks.

    Parameters:
        simulator_runtime (SimulatorRuntime): The runtime instance used for simulation
            management.

    Methods:
        reset(env_ids: List[int] = None) -> Tuple[List, List]: Resets specified environments
            and returns initial observations and task runtimes.
        step(action: List[Union[Dict, OrderedDict]]) -> Tuple[List, List, List, List, List]:
            Executes a single step in the environment using provided actions.
        get_dt(): Retrieves the simulation timestep (dt).
        get_observations() -> List | Any: Fetches observations from the simulation environment.
        close(): Closes the simulation environment.
        finished() -> bool: Checks if all tasks in the simulation are completed.

    Properties:
        runner: Provides access to the internal runner instance.
        is_render: Indicates whether the environment is in a renderable state.
        active_runtimes: Retrieves the currently active runtimes.
        simulation_runtime: Provides access to the active simulation runtime configuration.
        simulation_app: Retrieves the simulation app instance.
    """

    def __init__(self, simulator_runtime: SimulatorRuntime) -> None:
        self._render = None
        self._runtime = simulator_runtime

        from grutopia.core.runner import SimulatorRunner  # noqa E402.

        self._runner = SimulatorRunner(simulator_runtime=simulator_runtime)
        return

    def reset(self, env_ids: List[int] = None) -> Tuple[List, List]:
        """
        Resets the environments specified by the given environment IDs and returns the initial observations
        and task runtimes. If no environment IDs are provided, all environments will be reset. If no tasks
        are running after the reset, a log message is generated, and empty lists are returned.

        Parameters:
            env_ids (List[int]): A list of environment IDs to reset. If None, all environments will be reset.

        Returns:
            Tuple[List, List]: A tuple containing two lists: the initial observations and task runtimes for
            the reset environments. If no tasks are running, both lists will be empty.
        """
        obs, task_runtimes = self.runner.reset(env_ids=env_ids)

        if not task_runtimes or all(task_runtime is None for task_runtime in task_runtimes):
            log.info('All episodes have finished.')
            task_runtimes = []

        return obs, task_runtimes

    def step(self, action: List[Union[Dict, OrderedDict]]) -> Tuple[List, List, List, List, List]:
        """
        Perform a single step in the environment using the provided actions.

        This method takes a list of actions, validates its structure and length, then delegates
        the step execution to the internal runner. It computes and returns the observations,
        rewards, termination status, truncation status, and additional information.

        Args:
            action (List[Union[Dict, OrderedDict]]): A list of actions to be executed in the
                environment. Each action is either a dictionary or an ordered dictionary.

        Returns:
            Tuple[List, List, List, List, List]: A tuple containing the following elements:
                - obs (List): The observations resulting from the actions.
                - reward (List): The rewards obtained from the actions.
                - terminated (List): The termination status of the environments.
                - truncated (List): The truncation status of the environments.
                - info (List): Additional information about the step execution.
        """
        truncated = []
        info = []

        assert isinstance(action, list)
        assert len(action) == self._runtime.env_num

        obs, terminated_status, reward = self._runner.step(action)

        terminated = terminated_status

        return obs, reward, terminated, truncated, info

    @property
    def runner(self):
        """
        The runner property provides access to the internal runner instance.
        """
        return self._runner

    @property
    def is_render(self):
        """
        Get render state.
        """
        return self._render

    @property
    def active_runtimes(self):
        """
        Get active runtimes with env id as key.
        """
        return self._runtime.active_runtime()

    def get_dt(self):
        """
        Get dt of simulation environment.
        """
        return self._runner.dt

    def get_observations(self) -> List | Any:
        """
        Get observations from Isaac environment
        """
        _obs = self._runner.get_obs()
        return _obs

    def close(self):
        """Close the environment"""
        self._runtime.simulation_app.close()
        return

    @property
    def simulation_runtime(self):
        """Config of simulation environment"""
        return self._runtime.active_runtime()

    @property
    def simulation_app(self):
        """Simulation app instance"""
        return self._runtime.simulation_app

    def finished(self) -> bool:
        """Check if all tasks are finished"""
        return len(self._runner.current_tasks) == 0
