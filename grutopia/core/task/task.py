import os
import traceback
from abc import ABC
from functools import wraps
from typing import Any, Dict, Union

from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.tasks import BaseTask as OmniBaseTask
from omni.isaac.core.utils.prims import create_prim

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
    * contains scene/robots/objects.
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
        self.loaded = False
        for metric_config in runtime.metrics:
            self.metrics[metric_config.type] = create_metric(metric_config, self.runtime)

        from grutopia.core.task.reward import BaseReward, create_reward  # noqa

        self.reward: Union[BaseReward, None] = (
            create_reward(runtime.reward_setting, self) if runtime.reward_setting is not None else None
        )

    def load(self):
        """

        Loads the environment scene and initializes robots and objects.

        This method first checks if a scene asset path is defined in the runtime configuration. If so, it creates a scene using the provided path and specified parameters such as scaling and positioning. The scene is then populated with robots and objects based on the configurations stored within `self.runtime`.

        Raises:
        - Exceptions may be raised during file operations or USD scene creation, but specific exceptions are not documented here.

        Attributes Modified:
        - **self.robots**: A collection of initialized robots set up within the scene.
        - **self.objects**: A dictionary mapping object names to their respective initialized instances within the scene.
        - **self.loaded**: A boolean flag indicating whether the environment has been successfully loaded, set to `True` upon successful completion of this method.

        Logs:
        - Information about the initialized robots and objects is logged using the `log.info` method after successful setup.
        """
        if self.runtime.scene_asset_path is not None:
            source, prim_path = create_scene(
                os.path.abspath(self.runtime.scene_asset_path),
                prim_path_root=f'World/env_{self.runtime.env.env_id}/scene',
            )
            create_prim(
                prim_path,
                usd_path=source,
                scale=self.runtime.scene_scale,
                translation=[self.runtime.env.offset[idx] + i for idx, i in enumerate(self.runtime.scene_position)],
            )
        self.robots = init_robots(self.runtime, self._scene)
        self.objects = {}
        for obj in self.runtime.objects:
            _object = create_object(obj)
            _object.set_up_scene(self._scene)
            self.objects[obj.name] = _object

        log.info(self.robots)
        log.info(self.objects)
        self.loaded = True

    def set_up_scene(self, scene: Scene) -> None:
        """
        Adding assets to the stage as well as adding the encapsulated objects such as XFormPrim..etc
        to the task_objects happens here.

        Args:
            scene (Scene): [description]
        """
        self._scene = scene
        if not self.loaded:
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
        """

        Updates all metrics stored within the instance.

        Scans through the dictionary of metrics kept by the current instance,
        invoking the 'update' method on each one. This facilitates the aggregation
        or recalculation of metric values as needed.

        Note:
        This method does not return any value; its purpose is to modify the state
        of the metric objects internally.
        """
        for _, metric in self.metrics.items():
            metric.update()

    def calculate_metrics(self) -> dict:
        """

        Calculates and aggregates the results of all metrics registered within the instance.

        This method iterates over the stored metrics, calling their respective `calc` methods to compute
        the metric values. The computed values are then compiled into a dictionary, where each key corresponds
        to the metrics' name and each value is the result of the metric calculation.

        Returns:
            dict: A dictionary containing the calculated results of all metrics, with metric names as keys.

        Note:
            Ensure that all metrics added to the instance have a `calc` method implemented.

        Example Usage:
        ```python
        # Assuming `self.metrics` is populated with metric instances.
        results = calculate_metrics()
        print(results)
        # Output: {'metric1': 0.85, 'metric2': 0.92, 'metric3': 0.78}
        ```
        """
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
        """Called before calling a reset() on the world to removed temporary objects that were added during
        simulation for instance.
        """
        for obj in self.objects.keys():
            # Using try here because we want to ignore all exceptions
            try:
                self.scene.remove_object(obj)
            finally:
                log.info('objs cleaned.')
        for robot_name, robot in self.robots.items():
            # Using try here because we want to ignore all exceptions
            try:
                robot.cleanup()
                self.scene.remove_object(robot_name)
            finally:
                log.info('robots cleaned.')

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
