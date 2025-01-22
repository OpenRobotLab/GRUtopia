import json
import os
from typing import Dict, Optional, Tuple, Union

from omni.isaac.core import World
from omni.isaac.core.loggers import DataLogger
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.utils.stage import add_reference_to_stage  # noqa F401
from omni.physx.scripts import utils
from pxr import Usd  # noqa

# Init
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.scene import delete_prim_in_stage  # noqa F401
from grutopia.core.scene import create_object, create_scene  # noqa F401
from grutopia.core.task.task import BaseTask, create_task
from grutopia.core.util import AsyncRequest, log  # noqa F401
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
        self.metrics_results = {}
        self.metrics_save_path = simulator_runtime.task_runtime_manager.metrics_save_path
        if not os.path.exists(self.metrics_save_path) and self.metrics_save_path != 'console':
            os.makedirs(self.metrics_save_path)

        self._world: World = World(
            physics_dt=self.dt,
            rendering_dt=self.rendering_dt,
            stage_units_in_meters=1.0,
            sim_params={'use_fabric': self.use_fabric},
        )
        self._scene = self._world.scene
        self._stage = self._world.stage

        # Map task_name -> env_id:
        self.task_name_to_env_map = {}

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

    @property
    def current_tasks(self) -> dict[str, BaseTask]:

        return self._world._current_tasks

    def warm_up(self, steps=10, render=True):
        for _ in range(steps):
            self._world.step(render=render)

    def reload(self):
        self._world.reset()
        self._world.clear()
        self._world.stop()
        del self._world
        self._world = World(physics_dt=self.dt, rendering_dt=self.rendering_dt, stage_units_in_meters=1.0)
        self._scene = self._world.scene
        self._stage = self._world.stage
        self.reset()

    def step(
        self, actions: Union[Dict, None] = None, render: bool = True
    ) -> Tuple[Dict, Dict[str, bool], Dict[str, float]]:
        """
        Step function to advance the simulation environment by one time step.

        This method processes actions for active tasks, steps the simulation world,
        collects observations, updates metrics, and determines task terminations. It also
        handles rendering based on specified intervals.

        Args:
            actions (Union[Dict, None], optional): A dictionary mapping task names to
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
                - reward (Dict[str, float]): A dictionary that would理论上 contain rewards
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

        Caution:
            The snippet contains a `TODO` comment suggesting there's an aspect requiring
            attention related to "Key optimization interval," which isn't addressed in
            the docstring or the code shown.
        """
        """ ================ TODO: Key optimization interval ================= """
        terminated_status = {}
        reward = {}
        obs = {}

        for task_name, action_dict in actions.items():
            # terminated tasks will no longer apply action
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
        render = render and self.render_trigger > self.render_interval
        if self.render_trigger > self.render_interval:
            self.render_trigger = 0

        # Step
        self._world.step(render=render)

        # Get obs
        obs = self.get_obs()

        # Add render obs
        for task_name, task_obs in obs.items():
            for robot_name, robot_obs in task_obs.items():
                obs[task_name][robot_name]['render'] = render

        # update metrics
        for task in self.current_tasks.values():
            for metric in task.metrics.values():
                metric.update(obs[task.name])
            if task.is_done():
                self.finished_tasks.add(task.name)
                log.info(f'Task {task.name} finished.')
                if task.name not in self.metrics_results:
                    self.metrics_results[task.name] = task.calculate_metrics()
                self.metrics_results[task.name] = task.calculate_metrics()
                # TO DELETE
                self.metrics_results[task.name]['normally_end'] = True
                with open(
                    os.path.join(
                        self.metrics_save_path,
                        task.runtime.scene_asset_path.split('/')[-2]
                        + '_'
                        + '_'.join(task.runtime.extra['target'].split('/'))
                        + str(task.runtime.extra['episode_idx'])
                        + '.json',
                    ),
                    'w',
                ) as f:
                    f.write(json.dumps(self.metrics_results))
                exit()

            # finished tasks will no longer update metrics
            if task.name not in self.finished_tasks:
                for metric in task.metrics.values():
                    metric.update(obs[task.name])

        # update terminated_status and rewards
        for task_name in self.current_tasks.keys():
            terminated_status[task_name] = False
            if task_name in self.finished_tasks:
                terminated_status[task_name] = True
                reward[task_name] = -1
            else:
                r = self.current_tasks[task_name].reward
                reward[task_name] = r.calc() if r is not None else -1

        return obs, terminated_status, reward

    def get_obs(self) -> Dict:
        """
        Get obs
        Returns:
            Dict: obs from isaac sim.
        """
        obs = {}
        for task_name, task in self.current_tasks.items():
            obs[task_name] = task.get_observations()
        return obs

    def stop(self):
        """
        Stop all current operations and clean up the **World**
        """
        self._world.reset()
        self._world.clear()
        self._world.stop()

    def get_current_time_step_index(self) -> int:
        return self._world.current_time_step_index

    def reset(self, task: Optional[str] = None) -> Tuple[Dict, Union[TaskRuntime, None]]:
        """
        Reset the task.

        Args:
            task (str): A task name to reset. if task is None, it always means the reset is invoked for the first time
            before agents invoke step().

        Returns:
            Tuple[Dict, TaskRuntime]: A tuple of two values. The first is a dict of observations.  The second is a
            TaskRuntime object representing the new task runtime.
        """
        obs = self.get_obs()
        new_task_runtime: Union[TaskRuntime, None] = None

        if task is not None and task not in self.current_tasks:
            return obs, new_task_runtime

        # switch to next episodes
        # self.stop()
        new_task_runtime = self.next_episode(task)
        self.finished_tasks.discard(task)
        obs = self.get_obs()

        # finalize tasks
        if len(self.current_tasks) == 0:
            self._finalize()

        return obs, new_task_runtime

    def _finalize(self):
        """
        Finalize the tasks and do some post-processing.

        This function is called after all tasks are finished. Currently, it handles the metrics saving.

        """
        if len(self.current_tasks) == 0:
            # TODO metrics data post process(add hook in task)
            # print metrics data to console or save to file
            if self.metrics_save_path == 'console':
                # print to console
                log.info(json.dumps(self.metrics_results, indent=4))
            else:
                # save to file
                with open(self.metrics_save_path, 'w') as f:
                    f.write(json.dumps(self.metrics_results))

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
        log.info(f'Clear stage: /World/env_{self.task_name_to_env_map[task_name]}')
        clear_stage_by_prim_path(f'/World/env_{self.task_name_to_env_map[task_name]}')

    def next_episode(self, task_name: Optional[str] = None) -> Union[TaskRuntime, None]:
        """
        Switch to the next episode.

        This method cleanups a finished task specified by task name and then
        switches to the next task if exists.

        Args:
             task_name (Optional[str]): The task name of the finished task.

        Returns:
            TaskRuntime: new task runtime.

        Raises:
             RuntimeError: If the specified task_name is not found in the current tasks.
        """
        runtime_env = None
        if task_name is not None:
            if task_name not in self.current_tasks:
                raise RuntimeError(f'Task with task_name {task_name} not in current task_runtime_manager.')
            old_task = self.current_tasks[task_name]
            old_task_runtime = old_task.runtime
            self.clear_single_task(task_name)
            runtime_env = old_task_runtime.env

        next_task_runtime: Union[TaskRuntime, None] = self.task_runtime_manager.get_next_task_runtime(runtime_env)
        if next_task_runtime is None:
            self._reset_sim_context()
            return next_task_runtime

        env_id = next_task_runtime.env.env_id
        task = create_task(next_task_runtime, self._scene)
        self._world.add_task(task)
        task.set_up_scene(self._scene)
        self._reset_sim_context()
        task.post_reset()
        new_task_name = f'{next_task_runtime.name}'

        # Map task_name and env
        self.task_name_to_env_map[new_task_name] = str(env_id)

        # Log
        log.info('===================== episode ========================')
        log.info(f'Next episode: {new_task_name} at {str(env_id)}')
        log.info('======================================================')
        return next_task_runtime

    def _reset_sim_context(self):
        SimulationContext.reset(self._world, soft=False)
        self._world.scene._finalize(self._world.physics_sim_view)  # noqa

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
