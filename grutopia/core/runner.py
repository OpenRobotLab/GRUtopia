import json
from typing import Dict, List, Optional, Tuple, Union

from isaacsim.core.simulation_manager import SimulationManager
from omni.isaac.core import World
from omni.isaac.core.loggers import DataLogger
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.simulation_context import SimulationContext
from omni.physx.scripts import utils

# Init
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task.task import BaseTask, create_task
from grutopia.core.util import log
from grutopia.core.util.clear_task import clear_stage_by_prim_path


class SimulatorRunner:
    def __init__(self, simulator_runtime: SimulatorRuntime):
        self.task_runtime_manager = simulator_runtime.task_runtime_manager
        self._simulator_runtime = simulator_runtime
        physics_dt = (
            self._simulator_runtime.simulator.physics_dt
            if self._simulator_runtime.simulator.physics_dt is not None
            else None
        )
        self.rendering_dt = (
            self._simulator_runtime.simulator.rendering_dt
            if self._simulator_runtime.simulator.rendering_dt is not None
            else None
        )
        physics_dt = eval(physics_dt) if isinstance(physics_dt, str) else physics_dt
        self.rendering_dt = eval(self.rendering_dt) if isinstance(self.rendering_dt, str) else self.rendering_dt
        self.dt = physics_dt
        self.use_fabric = self._simulator_runtime.simulator.use_fabric
        log.info(
            f'simulator params: physics dt={self.dt}, rendering dt={self.rendering_dt}, use_fabric={self.use_fabric}'
        )

        self.metrics_config = None
        self.metrics_save_path = simulator_runtime.task_runtime_manager.metrics_save_path
        if self.metrics_save_path != 'console':
            try:
                with open(self.metrics_save_path, 'w'):
                    pass
            except Exception as e:
                log.error(f'Can not create result file at {self.metrics_save_path}.')
                raise e

        self._world: World = World(
            physics_dt=self.dt,
            rendering_dt=self.rendering_dt,
            stage_units_in_meters=1.0,
            sim_params={'use_fabric': self.use_fabric},
        )
        self._scene = self._world.scene
        self._stage = self._world.stage

        # Map task_name -> env_id:
        self.task_name_to_env_id_map = {}
        self.env_id_to_task_name_map = {}

        # finished_tasks contains all the finished tasks in current tasks dict
        self.finished_tasks = set()

        self.render_interval = (
            self._simulator_runtime.simulator.rendering_interval
            if self._simulator_runtime.simulator.rendering_interval is not None
            else 5
        )
        log.info(f'rendering interval: {self.render_interval}')
        self.render_trigger = 0
        self.loop = False
        self._render = False

    @property
    def current_tasks(self) -> dict[str, BaseTask]:
        return self._world._current_tasks

    def warm_up(self, steps: int = 10, render: bool = True, physics: bool = True):
        """
        Warm up the simulation by running a specified number of steps.

        Args:
            steps (int): The number of warm-up steps to perform. Defaults to 10.
            render (bool): Whether to render the scene during warm-up. Defaults to True.
            physics (bool): Whether to enable physics during warm-up. Defaults to True.

        Raises:
            ValueError: If both `render` and `physics` are set to False, or if `steps` is less than or equal to 0.
        """
        if not render and not physics:
            raise ValueError('both `render` and `physics` are set to False')
        if steps <= 0:
            raise ValueError('steps` is less than or equal to 0')
        if physics:
            for _ in range(steps):
                self._world.step(render=render)
        else:
            for _ in range(steps):
                SimulationContext.render(self._world)

    def step(
        self, actions: Union[List[Dict], None] = None, render: bool = True
    ) -> Tuple[List[Dict], List[bool], List[float]]:
        """
        Step function to advance the simulation environment by one time step.

        This method processes actions for active tasks, steps the simulation world,
        collects observations, updates metrics, and determines task terminations. It also
        handles rendering based on specified intervals.

        Args:
            actions (Union[List[Dict], None], optional): A dictionary mapping task names to
                another dictionary of robot names and their respective actions. If None,
                no actions are applied. Defaults to None.
            render (bool, optional): Flag indicating whether to render the simulation
                at this step. True triggers rendering if the render interval is met.
                Defaults to True.

        Returns:
            Tuple[Dict, Dict[str, bool], Dict[str, float]]:
                - obs (Dict): A dictionary containing observations for each task,
                  further divided by robot names and their observation data.
                - terminated_status (Dict[str, bool]): A dictionary mapping task names
                  to boolean values indicating whether the task has terminated.
                - reward (Dict[str, float]): A dictionary that would contain rewards
                  for each task or robot; however, the actual return and computation of
                  rewards is not shown in the provided code snippet.

        Raises:
            Exception: If an error occurs when applying an action to a robot, the
                exception is logged and re-raised, providing context about the task,
                robot, and current tasks state.

        Notes:
            - The `_world.step()` method advances the simulation, optionally rendering
              the environment based on the `render` flag and the render interval.
            - `get_obs()` is a method to collect observations from the simulation world,
              though its implementation details are not shown.
            - Metrics for each task are updated, and upon task completion, results are
              saved to a JSON file. This includes a flag 'normally_end' set to True,
              which seems to indicate normal termination of the task.
            - The function also manages a mechanism to prevent further action application
              and metric updates for tasks that have been marked as finished.
        """
        """ ================ TODO: Key optimization interval ================= """
        terminated_status = []
        reward = []

        for env_id, action_dict in enumerate(actions):
            # terminated tasks will no longer apply action
            if env_id not in self.env_id_to_task_name_map:
                continue

            task_name = self.env_id_to_task_name_map[env_id]

            if task_name in self.finished_tasks:
                continue

            if task_name not in self.current_tasks:
                continue

            task = self.current_tasks.get(task_name)
            for name, action in action_dict.items():
                if name in task.robots:
                    try:
                        task.robots[name].apply_action(action)
                    except Exception as e:
                        log.error('task_name     : %s', task_name)
                        log.error('robot_name    : %s', name)
                        log.error('current_tasks : %s', [i for i in self.current_tasks.keys()])
                        raise e

        self.render_trigger += 1
        self._render = render and self.render_trigger > self.render_interval
        if self.render_trigger > self.render_interval:
            self.render_trigger = 0

        # Step
        self._world.step(render=self._render)

        # Get obs
        obs = self.get_obs()

        # update metrics
        for task in self.current_tasks.values():
            if task.is_done():
                self.finished_tasks.add(task.name)
                log.info(f'Task {task.name} finished.')
                metrics_results = task.calculate_metrics()
                metrics_results['normally_end'] = True
                if self.metrics_save_path == 'console':
                    print(json.dumps(metrics_results, indent=4))
                elif self.metrics_save_path == 'none':
                    pass
                else:
                    with open(self.metrics_save_path, 'a') as f:
                        f.write(json.dumps(metrics_results))
                        f.write('\n')

            # finished tasks will no longer update metrics
            if task.name not in self.finished_tasks:
                for metric in task.metrics.values():
                    metric.update(obs[self.task_name_to_env_id_map[task.name]])

        # update terminated_status and rewards
        for env_id in range(self.task_runtime_manager.env_num):
            # terminated tasks will no longer apply action

            if env_id not in self.env_id_to_task_name_map:
                terminated_status.append(True)
                reward.append(-1)
                continue

            task_name = self.env_id_to_task_name_map[env_id]

            if task_name in self.finished_tasks:
                terminated_status.append(True)
                reward.append(-1)
            else:
                terminated_status.append(False)
                r = self.current_tasks[task_name].reward
                reward.append(r.calc() if r is not None else -1)

        return obs, terminated_status, reward

    def get_obs(self) -> List[Dict | None]:
        """
        Get obs
        Returns:
            List[Dict]: obs from isaac sim.
        """
        obs = {}
        for task_name, task in self.current_tasks.items():
            obs[task_name] = task.get_observations()
        # Add render obs
        for task_name, task_obs in obs.items():
            for robot_name, robot_obs in task_obs.items():
                obs[task_name][robot_name]['render'] = self._render

        _obs = []
        for env_id in range(self._simulator_runtime.env_num):
            if env_id in self.env_id_to_task_name_map:
                _obs.append(obs[self.env_id_to_task_name_map[env_id]])
            else:
                _obs.append(None)
        return _obs

    def stop(self):
        """
        Stop all current operations and clean up the **World**
        """
        self._world.reset()
        self._world.clear()
        self._world.stop()

    def get_current_time_step_index(self) -> int:
        return self._world.current_time_step_index

    def reset(self, env_ids: Optional[List[int]] = None) -> Tuple[List, List]:
        """
        Resets the environment for the given environment IDs or initializes it if no IDs are provided.
        This method handles resetting the simulation context, generating new task runtimes, and finalizing
        tasks when necessary. It supports partial resets for specific environments and ensures proper
        handling of task transitions.

        Args:
            env_ids (Optional[List[int]]): A list of environment IDs to reset. If None, all environments
                are reset or initialized based on the current state.

        Returns:
            Tuple[List, List]: A tuple containing two lists. The first list contains observations for
                the reset environments, and the second list contains the new task runtimes.

        Raises:
            ValueError: If the provided `env_ids` are invalid or don't correspond to any existing tasks.
            RuntimeError: If the simulation context fails to reset or initialize properly.

        Notes:
            - If `env_ids` is None and there are current tasks, all environments are reset.
            - If `env_ids` is None and there are no current tasks, the simulation context is initialized.
            - Observations are collected only for environments that are reset or initialized.
            - Tasks corresponding to the reset environments are transitioned to new episodes.
        """
        new_task_runtimes = []

        if env_ids is None and self.current_tasks:
            # reset
            log.info('==================== reset all env ====================')
            env_ids = [i for i in range(self._simulator_runtime.env_num)]

        if env_ids is None:
            # init
            log.info('===================== init reset =====================')
            SimulationContext.reset(self._world, soft=False)
            new_task_runtimes = self._next_episodes()
        else:
            # switch to next episodes
            env_to_reset = [env_id for env_id in env_ids if env_id in self.env_id_to_task_name_map]

            if not env_to_reset:
                log.warning(f'No env to reset: {env_ids}.')
                return [None for _ in env_ids], [None for _ in env_ids]

            tasks = [self.env_id_to_task_name_map[env_id] for env_id in env_to_reset]
            _task_runtimes = self._next_episodes(tasks)
            for env_id in env_ids:
                if env_id in self.env_id_to_task_name_map:
                    new_task_runtimes.append(_task_runtimes.pop(0))
                else:
                    new_task_runtimes.append(None)
            [self.finished_tasks.discard(task) for task in tasks]

        all_obs = self.get_obs()
        obs = [all_obs[i] for i in env_ids] if env_ids else all_obs

        if not self.current_tasks:
            # finished
            self._finalize()

        return obs, new_task_runtimes

    def _finalize(self):
        """
        Finalize the tasks and do some post-processing.
        """
        pass

    def world_clear(self):
        self._world.clear()

    def clear_single_task(self, task_name: str):
        """
        Clear single task with task_name

        Args:
            task_name (str): Task name to clear.

        """
        if task_name not in self.current_tasks:
            log.warning(f'Clear task {task_name} fail. The task {task_name} is not in current_tasks.')
            return
        old_task = self.current_tasks[task_name]
        old_task.cleanup()
        del self.current_tasks[task_name]
        self._world._task_scene_built = False
        self._world._data_logger = DataLogger()
        log.info(f'Clear stage: /World/env_{self.task_name_to_env_id_map[task_name]}')
        clear_stage_by_prim_path(f'/World/env_{self.task_name_to_env_id_map[task_name]}')

    def _next_episodes(self, reset_tasks: List[str] = None) -> List[TaskRuntime]:
        """
        Switch tasks that need to be reset to the next episode.

        This method handles cleaning-up tasks, resetting sim backend, creating new tasks, and
        restoring states for non-reset environments.

        Args:
            reset_tasks (List[str]): a list of task names that need to be reset. If None, the method
                initializes all environments without resetting specific tasks.

        Returns:
            List[TaskRuntime]: a list of TaskRuntime objects representing the next set of tasks to be
                executed in the simulation.

        Raises:
            RuntimeError: If a task specified in `reset_tasks` isn't found in the current tasks.
        """
        runtime_envs: Union[None, TaskRuntime.env] = []
        next_task_runtimes = []

        if reset_tasks:
            # recycling env_id
            for task_name in reset_tasks:
                if task_name not in self.current_tasks:
                    raise RuntimeError(f'Task with task_name {task_name} not in `current_tasks`.')
                old_task = self.current_tasks[task_name]
                old_task_runtime: TaskRuntime = old_task.runtime
                runtime_envs.append(old_task_runtime.env)

            # save the state of envs that need to be kept
            for _task_name, task in self.current_tasks.items():
                if _task_name in reset_tasks:
                    continue
                task.save_info()

            # clean up tasks that need to reset
            for task_name in reset_tasks:
                self.clear_single_task(task_name)

            # clear all rigid bodies in scene register and physics backend
            for task in self.current_tasks.values():
                task.clear_rigid_bodies()

            SimulationManager._on_stop('reset')
        else:
            # init
            SimulationManager._on_stop('reset')
            runtime_envs = [None for _ in range(self._simulator_runtime.env_num)]

        # get next_task_runtimes
        for runtime_env in runtime_envs:
            next_task_runtime: Union[TaskRuntime, None] = self.task_runtime_manager.get_next_task_runtime(runtime_env)

            if next_task_runtime is None:
                del self.env_id_to_task_name_map[runtime_env.env_id]
            next_task_runtimes.append(next_task_runtime)

        # create tasks with next_task_runtimes
        _new_tasks = []
        _new_tasks_names = []
        for next_task_runtime in next_task_runtimes:
            if next_task_runtime is None:
                continue
            task = create_task(next_task_runtime, self._scene)
            self._world.add_task(task)
            _new_tasks.append(task)
            _new_tasks_names.append(task.name)
            task.set_up_scene(self._scene)

        # create sim_view
        SimulationManager._create_simulation_view('reset')

        # restore the state of envs that haven't been reset
        if reset_tasks:
            for t in self.current_tasks.values():
                if t.name in _new_tasks_names:
                    continue
                t.restore_info()

        self._scene._finalize(self._world.physics_sim_view)  # noqa
        self._scene.post_reset()

        # post_reset for new tasks
        for task in _new_tasks:
            task.post_reset()

        # map task_name and env_id of new tasks
        for next_task_runtime in next_task_runtimes:
            if next_task_runtime is None:
                continue
            self.task_name_to_env_id_map[next_task_runtime.name] = next_task_runtime.env.env_id
            self.env_id_to_task_name_map[next_task_runtime.env.env_id] = next_task_runtime.name

        # log new episodes
        log.info('===================== episodes ========================')
        for next_task_runtime in next_task_runtimes:
            if next_task_runtime is None:
                continue
            log.info(f'Next episode: {next_task_runtime.name} at {str(next_task_runtime.env.env_id)}')
        log.info('======================================================')
        return next_task_runtimes

    def get_obj(self, name: str) -> XFormPrim:
        return self._world.scene.get_object(name)

    def remove_collider(self, prim_path: str):
        build = self._world.stage.GetPrimAtPath(prim_path)
        if build.IsValid():
            utils.removeCollider(build)

    def add_collider(self, prim_path: str):
        build = self._world.stage.GetPrimAtPath(prim_path)
        if build.IsValid():
            utils.setCollider(build, approximationShape=None)
