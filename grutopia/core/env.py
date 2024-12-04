from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.runtime.task_runtime import TaskRuntime


class Env:
    """
    Vector Env
    ----------------------------------------------------------------------
    """

    def __init__(self, simulator_runtime: SimulatorRuntime) -> None:
        self._simulation_runtime = simulator_runtime
        self._render = None
        # Setup Multitask Env Parameters
        self.env_map = {}
        self.obs_map = {}

        self.runtime = simulator_runtime
        self.env_num = simulator_runtime.env_num
        self._column_length = int(np.sqrt(self.env_num))

        from grutopia.core.runner import SimulatorRunner  # noqa E402.

        self._runner = SimulatorRunner(simulator_runtime=simulator_runtime)

        return

    @property
    def runner(self):
        return self._runner

    @property
    def is_render(self):
        return self._render

    @property
    def active_runtimes(self):
        return self.runtime.active_runtime()

    def get_dt(self):
        """
        Get dt of simulation environment.
        Returns:
            dt.
        """
        return self._runner.dt

    def step(self, actions: Dict[str, Dict[str, Any]]) -> Tuple[Dict, Dict[str, bool]]:
        """
        run step with given action(with isaac step)

        Args:
            actions (Dict[str, Dict[str, Any]]): action(with isaac step)

        Returns:
            Tuple[Dict, Dict[str, bool]]: A tuple of two values. The first is a dict of observations.
            The second is a dict that maps task name to termination status.
        """
        return self._runner.step(actions)

    def reset(self, task: Optional[str] = None) -> Tuple[Dict, TaskRuntime]:
        """
        Reset the task.

        Args:
            task (str): A task name to reset. if task is None, it always means the reset is invoked for the first time
            before agents invoke step(). The function will return the initial obs from the world.

        Returns:
            Tuple[Dict, TaskRuntime]: A tuple of two values. The first is a dict of observations.  The second is a
                TaskRuntime object representing the new task runtime.
        """
        return self._runner.reset(task)

    def vector_reset(self) -> Tuple[Dict, List[TaskRuntime]]:
        """
        Reset all sub envs. It is always invoked for initialization before stepping through the world

        Returns:
            Tuple[Dict, TaskRuntime]: A tuple of two values. The first is a dict of observations. The second is a list of TaskRuntime.
        """

        obs = {}
        task_runtime_list = []

        for _ in range(self.env_num):
            obs, new_runtime = self.reset()
            if new_runtime is not None:
                task_runtime_list.append(new_runtime)

        self.runner.warm_up()
        return obs, task_runtime_list

    def get_observations(self) -> Dict[str, Dict[str, Any]]:
        """
        Get observations from Isaac environment
        Returns:
            Dict[str, Dict[str, Any]]: observations
        """
        _obs = self._runner.get_obs()
        return _obs

    def render(self, mode='human'):
        pass

    def close(self):
        """close the environment"""
        self._simulation_runtime.simulation_app.close()
        return

    @property
    def simulation_runtime(self):
        """config of simulation environment"""
        return self._simulation_runtime.active_runtime()

    @property
    def simulation_app(self):
        """simulation app instance"""
        return self._simulation_runtime.simulation_app

    def finished(self) -> bool:
        """check if all tasks are finished"""
        return len(self._runner.current_tasks) == 0
