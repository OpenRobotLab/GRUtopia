import json
from typing import Dict, List, Optional, Tuple, Union

from omni.isaac.core import World
from omni.isaac.core.loggers import DataLogger
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.simulation_context import SimulationContext
from omni.isaac.core.utils.stage import add_reference_to_stage  # noqa F401
from omni.physx.scripts import utils
from pxr import Usd  # noqa

# Init
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.runtime.task_runtime import BaseTaskRuntimeManager, TaskRuntime
from grutopia.core.scene import delete_prim_in_stage  # noqa F401
from grutopia.core.scene import create_object, create_scene  # noqa F401
from grutopia.core.task.task import BaseTask, create_task
from grutopia.core.util import log
from grutopia.core.util.clear_task import clear_stage_by_prim_path


class SimulatorRunner:

    def __init__(self, simulator_runtime: SimulatorRuntime):
        self.task_runtime_manager = simulator_runtime.task_runtime_manager

        self._simulator_runtime = simulator_runtime
        physics_dt = self._simulator_runtime.simulator.physics_dt if self._simulator_runtime.simulator.physics_dt is not None else None
        rendering_dt = self._simulator_runtime.simulator.rendering_dt if self._simulator_runtime.simulator.rendering_dt is not None else None
        physics_dt = eval(physics_dt) if isinstance(physics_dt, str) else physics_dt
        rendering_dt = eval(rendering_dt) if isinstance(rendering_dt, str) else rendering_dt
        self.dt = physics_dt
        log.debug(f'Simulator physics dt: {self.dt}')

        self.metrics_config = None
        self.metrics_results = {}
        self.metrics_save_path = (simulator_runtime.task_runtime_manager.metrics_save_path)

        self._world = World(physics_dt=self.dt, rendering_dt=rendering_dt, stage_units_in_meters=1.0)
        self._scene = self._world.scene
        self._stage = self._world.stage

        # Map task_name -> env_id:
        self.task_name_to_env_map = {}

        # finished_tasks contains all the finished tasks in current tasks dict
        self.finished_tasks = set()

        self.render_interval = (self._simulator_runtime.simulator.rendering_interval
                                if self._simulator_runtime.simulator.rendering_interval is not None else 5)
        log.info(f'rendering interval: {self.render_interval}')
        self.render_trigger = 0

    @property
    def current_tasks(self) -> dict[str, BaseTask]:

        return self._world._current_tasks

    def warm_up(self, steps=10, render=True):
        for _ in range(steps):
            self._world.step(render=render)

    def init_tasks(self, tasks_runtime_manager: BaseTaskRuntimeManager):
        self.task_runtime_manager = tasks_runtime_manager
        for config in tasks_runtime_manager.active_runtimes.values():
            task = create_task(config, self._scene)
            self._world.add_task(task)
            self.task_name_to_env_map[task.runtime.name] = str(task.runtime.env.env_id)

        self._world.reset()
        self.warm_up()

    def reload_tasks(self, runtimes: List[TaskRuntime]):
        """
        Reload tasks with list of TaskRuntimes.

        Args:
            runtimes (List[TaskRuntime]): Runtimes of task to be reloaded.
        """
        tasks = []
        for runtime in runtimes:
            task = create_task(runtime, self._scene)
            if task.name in self.current_tasks:
                self.clear_single_task(task.name)
            else:
                continue
            self._world.add_task(task)
            tasks.append(task)
        for task in tasks:
            task.set_up_scene(self._scene)
            task.cleanup()
        if tasks:
            SimulationContext.reset(self._world, soft=False)
            self._world.scene._finalize(self._world.physics_sim_view)
        for task in tasks:
            task.post_reset()
            self.finished_tasks.discard(task.name)

    def step(self, actions: Union[Dict, None] = None, render: bool = True) -> Tuple[Dict, Dict[str, bool]]:
        """
        Assemble Actions to Robots of each task, and run Isaac's world step.
        The method will return termination status of each tasks, agents are expected to reset terminated task before continuing next step.

        Args:
            actions (Dict): Action dict. task_name as key.
            render (bool): render sign.

        Returns:
            Tuple[Dict, Dict[str, bool]]: A tuple of two values. The first is a dict of observations.
            The second is a dict that maps task name to termination status.
        """
        """ ================ TODO: Key optimization interval ================= """
        terminated_status = {}
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

        # update metrics
        for task in self.current_tasks.values():
            if task.is_done():
                self.finished_tasks.add(task.name)
                log.info(f'Task {task.name} finished.')
                if task.name not in self.metrics_results:
                    self.metrics_results[task.name] = task.calculate_metrics()

            # finished tasks will no longer update metrics
            if task.name not in self.finished_tasks:
                for metric in task.metrics.values():
                    metric.update(obs[task.name])

        # update terminated_status
        for task_name in self.current_tasks.keys():
            terminated_status[task_name] = False
            if task_name in self.finished_tasks:
                terminated_status[task_name] = True

        return obs, terminated_status

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

    def get_current_time_step_index(self) -> int:
        return self._world.current_time_step_index

    def reset(self, task: Optional[str] = None) -> Tuple[Dict, TaskRuntime]:
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
        new_task_runtime = None

        if task is not None and task not in self.current_tasks:
            return obs, new_task_runtime

        # switch to next episodes
        new_task_runtime = self.next_episode(task)
        self.finished_tasks.discard(task)
        obs = self.get_obs()

        # finalize tasks
        if len(self.current_tasks) == 0:
            self._finalize()

        return obs, new_task_runtime

    def _finalize(self):
        """
        Finalize the tasks and do some post processing.

        This function is called after all tasks are finished. Currently it handle the metrics saving.

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
        old_task.clean_scene()
        del self.current_tasks[task_name]
        self._world._task_scene_built = False
        self._world._data_logger = DataLogger()
        log.info(f'Clear stage: /World/env_{self.task_name_to_env_map[task_name]}')
        clear_stage_by_prim_path(f'/World/env_{self.task_name_to_env_map[task_name]}')

    def next_episode(self, task_name: Optional[str] = None) -> TaskRuntime:
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
            self.clear_single_task(task_name)
            runtime_env = old_task.runtime.env

        next_task_runtime: Union[TaskRuntime, None] = (self.task_runtime_manager.get_next_task_runtime(runtime_env))
        if next_task_runtime is None:
            self._reset_sim_context()
            return next_task_runtime

        env_id = next_task_runtime.env.env_id
        task = create_task(next_task_runtime, self._scene)
        self._world.add_task(task)
        task.set_up_scene(self._scene)
        task.cleanup()
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
