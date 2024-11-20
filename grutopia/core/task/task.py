import os
import traceback
from abc import ABC
from functools import wraps
from typing import Any, Dict, Union

from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.tasks import BaseTask as OmniBaseTask
from omni.isaac.core.utils.prims import create_prim

from grutopia.core.datahub import DataHub
from grutopia.core.robot import init_robots
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.scene import create_object, create_scene
from grutopia.core.task.metric import BaseMetric, create_metric
from grutopia.core.util import log


class BaseTask(OmniBaseTask, ABC):
    """
    wrap of omniverse isaac sim's base task

    * enable register for auto register task
    * contains scene/robots/objs.
    """
    tasks = {}

    def __init__(self, runtime: TaskRuntime, scene: Scene):
        self.objects = None
        self.robots: Union[Dict[str, BaseRobot], None] = None
        name = runtime.name
        offset = runtime.env.offset
        super().__init__(name=name, offset=offset)
        self._scene = scene
        self.runtime = runtime

        self.metrics: Dict[str, BaseMetric] = {}
        self.steps = 0
        self.work = True

        for metric_config in runtime.metrics:
            self.metrics[metric_config.type] = create_metric(metric_config, self.runtime)

    def load(self):
        """
        Load assets to scene.
        """
        if self.runtime.scene_asset_path is not None:
            source, prim_path = create_scene(os.path.abspath(self.runtime.scene_asset_path),
                                             prim_path_root=f'World/env_{self.runtime.env.env_id}/scene')
            create_prim(
                prim_path,
                usd_path=source,
                scale=self.runtime.scene_scale,
                translation=[self.runtime.env.offset[idx] + i for idx, i in enumerate(self.runtime.scene_position)])

        self.robots = init_robots(self.runtime, self._scene)
        self.objects = {}
        for obj in self.runtime.objects:
            _object = create_object(obj)
            _object.set_up_scene(self._scene)
            self.objects[obj.name] = _object

        log.info(self.robots)
        log.info(self.objects)

    def set_up_scene(self, scene: Scene) -> None:
        """
        Adding assets to the stage as well as adding the encapsulated objects such as XFormPrim..etc
        to the task_objects happens here.

        Args:
            scene (Scene): [description]
        """
        self._scene = scene
        self.load()

    def clean_scene(self):
        """
        Clean scene when reload
        """
        for obj in self.objects.keys():
            # Using try here because we want to ignore all exceptions
            try:
                self.scene.remove_object(obj)
            finally:
                log.info('objs cleaned.')
        for robot in self.robots.keys():
            # Using try here because we want to ignore all exceptions
            try:
                self.scene.remove_object(robot)
            finally:
                log.info('robots cleaned.')

    def get_observations(self) -> Dict[str, Any]:
        """
        Returns current observations from the objects needed for the behavioral layer.

        Return:
            Dict[str, Any]: observation of robots in this task
        """
        if not self.work:
            return {}
        obs = {}
        for robot_name, robot in self.robots.items():
            try:
                _obs = robot.get_obs()
                if _obs:
                    obs[robot_name] = _obs
            except Exception as e:
                log.error(self.name)
                log.error(e)
                traceback.print_exc()
                return {}
        return obs

    def update_metrics(self):
        for _, metric in self.metrics.items():
            metric.update()

    def calculate_metrics(self) -> dict:
        metrics_res = {}
        for name, metric in self.metrics.items():
            metrics_res[name] = metric.calc()

        return metrics_res

    def is_done(self) -> bool:
        """
        Returns True of the task is done. The result should be decided by the state of the task.
        """
        raise NotImplementedError

    def individual_reset(self):
        """
        Reload this task individually without reloading whole world.
        """
        raise NotImplementedError

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """called before stepping the physics simulation.

        Args:
            time_step_index (int): [description]
            simulation_time (float): [description]
        """
        self.steps += 1
        return

    def post_reset(self) -> None:
        """Calls while doing a .reset() on the world."""
        self.steps = 0
        for robot in self.robots.values():
            robot.post_reset()
        return

    def cleanup(self) -> None:
        pass

    @classmethod
    def register(cls, name: str):
        """
        Register a task with its name(decorator).
        Args:
            name(str): name of the task
        """

        def decorator(tasks_class):
            cls.tasks[name] = tasks_class

            @wraps(tasks_class)
            def wrapped_function(*args, **kwargs):
                return tasks_class(*args, **kwargs)

            return wrapped_function

        return decorator


def create_task(config: TaskRuntime, scene: Scene):
    task_cls: BaseTask = BaseTask.tasks[config.type](config, scene)
    return task_cls
