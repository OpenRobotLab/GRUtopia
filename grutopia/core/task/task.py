import os
import traceback
from abc import ABC, abstractmethod
from functools import wraps
from typing import Any, Dict

from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.tasks import BaseTask as OmniBaseTask
from omni.isaac.core.utils.prims import create_prim

from grutopia.core.config import TaskUserConfig
from grutopia.core.robot import init_robots
from grutopia.core.scene import create_object, create_scene
from grutopia.core.task.metric import BaseMetric, create_metric
from grutopia.core.util import log


class BaseTask(OmniBaseTask, ABC):
    """
    wrap of omniverse isaac sim's base task

    * enable register for auto register task
    * contains robots
    """
    tasks = {}

    def __init__(self, config: TaskUserConfig, scene: Scene):
        self.objects = None
        self.robots = None
        name = config.name
        offset = config.offset
        super().__init__(name=name, offset=offset)
        self._scene = scene
        self.config = config

        self.metrics: dict[str, BaseMetric] = {}
        self.steps = 0
        self.work = True

        for metric_config in config.metrics:
            self.metrics[metric_config.name] = create_metric(metric_config)

    def load(self):
        if self.config.scene_asset_path is not None:
            source, prim_path = create_scene(os.path.abspath(self.config.scene_asset_path),
                                             prim_path_root=f'World/env_{self.config.env_id}/scene')
            create_prim(prim_path,
                        usd_path=source,
                        scale=self.config.scene_scale,
                        translation=[self.config.offset[idx] + i for idx, i in enumerate(self.config.scene_position)])

        self.robots = init_robots(self.config, self._scene)
        self.objects = {}
        for obj in self.config.objects:
            _object = create_object(obj)
            _object.set_up_scene(self._scene)
            self.objects[obj.name] = _object
        log.info(self.robots)
        log.info(self.objects)

    def set_up_scene(self, scene: Scene) -> None:
        self._scene = scene
        self.load()

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
                obs[robot_name] = robot.get_obs()
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

    @abstractmethod
    def is_done(self) -> bool:
        """
        Returns True of the task is done.

        Raises:
            NotImplementedError: this must be overridden.
        """
        raise NotImplementedError

    def individual_reset(self):
        """
        reload this task individually without reloading whole world.
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


def create_task(config: TaskUserConfig, scene: Scene):
    task_cls = BaseTask.tasks[config.type]
    return task_cls(config, scene)
