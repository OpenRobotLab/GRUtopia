from typing import Any, Dict

import numpy as np

from grutopia.core.runtime import SimulatorRuntime


class Env:
    """
    Env base class. All envs should inherit from this class(or subclass).
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
        return self.runtime.task_runtime_manager.active_runtimes

    def get_dt(self):
        """
        Get dt of simulation environment.
        Returns:
            dt.
        """
        return self._runner.dt

    def step(self, actions: Dict[str, Dict[str, Any]]) -> Dict[str, Dict[str, Any]]:
        """
        run step with given action(with isaac step)


        format of input actions :
        -------------------------

        .. code-block::

            {
                "task_1_name":
                {
                    "robot_1": {
                        "controller_1": [1, 2, 3],
                        "controller_2": [1, 2, 3]
                    },
                    "robot_2": {
                        "controller_1": [1, 2, 3],
                        "controller_2": [1, 2, 3]
                    },
                },
                "task_2_name":
                {
                    "robot_1": {
                        "controller_1": [1, 2, 3],
                        "controller_2": [1, 2, 3]
                    },
                    "robot_2": {
                        "controller_1": [1, 2, 3],
                        "controller_2": [1, 2, 3]
                    },
                },
            }


        format of actions send to runner:
        ---------------------------------

        .. code-block::


            {
                "task_1_name":
                {
                    "robot_1_1": {
                        "controller_1": [1, 2, 3],
                        "controller_2": [1, 2, 3]
                    },
                    "robot_2_1": {
                        "controller_1": [1, 2, 3],
                        "controller_2": [1, 2, 3]
                    },
                },
                "task_2_name":
                {
                    "robot_1_2": {
                        "controller_1": [1, 2, 3],
                        "controller_2": [1, 2, 3]
                    },
                    "robot_2_2": {
                        "controller_1": [1, 2, 3],
                        "controller_2": [1, 2, 3]
                    },
                },
            }

        Intro:
        ------

        Because robots' name in all tasks need to be unique.
        And the suffix of robot_name is `env_id`.

        Args:
            actions (Dict[str, Dict[str, Any]]): action(with isaac step)

        Returns:
            Dict[str, Dict[str, Any]]: observations(with isaac step)
        """
        _actions = {}
        for task_name, action in actions.items():
            _action = {}
            env_id = self.runner.task_name_to_env_map[task_name]
            for k, v in action.items():
                _action[f'{k}_{env_id}'] = v
            _actions[task_name] = _action
        action_after_reshape = {
            self.active_runtimes[self.runner.task_name_to_env_map[task_name]].name: action
            for task_name, action in _actions.items()
            if self.runner.task_name_to_env_map[task_name] in self.runtime.task_runtime_manager.active_runtimes
        }

        _, finish = self._runner.step(action_after_reshape)
        observations = self.get_observations()

        if finish:
            self._simulation_runtime.simulation_app.close()

        return observations

    def reset(self):
        """reset the environment(use isaac word reset)"""
        self._runner.reset()
        return self.get_observations(), {}

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
        return self._simulation_runtime

    @property
    def simulation_app(self):
        """simulation app instance"""
        return self._simulation_runtime.simulation_app
