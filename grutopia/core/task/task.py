import os
import traceback
from abc import ABC
from functools import wraps
from typing import Any, Dict, Union

from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.scenes.scene import Scene
from omni.isaac.core.tasks import BaseTask as OmniBaseTask
from omni.isaac.core.utils.prims import create_prim
from pxr import Usd

from grutopia.core.robot import init_robots
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.scene import create_object, create_scene
from grutopia.core.task.metric import BaseMetric, create_metric
from grutopia.core.util import log
from grutopia.core.util.physics_status_util import (
    get_rigidbody_status,
    set_rigidbody_status,
)


class BaseTask(OmniBaseTask, ABC):
    """
    wrap of omniverse isaac sim's base task

    * enable register for auto register task
    * contains scene/robots/objects.
    """

    tasks = {}

    def __init__(self, runtime: TaskRuntime, scene: Scene):
        self.scene_prim = None
        self.objects = None
        self.robots: Union[Dict[str, BaseRobot], None] = None
        name = runtime.name
        self.env_id = runtime.env.env_id
        self.offset = runtime.env.offset
        super().__init__(name=name, offset=self.offset)
        self._scene = scene
        self.scene_rigid_bodies = {}
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
            scene_prim = create_prim(
                prim_path,
                usd_path=source,
                scale=self.runtime.scene_scale,
                translation=[self.runtime.env.offset[idx] + i for idx, i in enumerate(self.runtime.scene_position)],
            )
            self.scene_prim = scene_prim
            for prim in Usd.PrimRange.AllPrims(scene_prim):
                if prim.GetAttribute('physics:rigidBodyEnabled'):
                    log.debug(f'[BaseTask.load] found rigid body at path: {prim.GetPath()}')
                    _rb = RigidPrim(str(prim.GetPath()))
                    self.scene_rigid_bodies[str(prim.GetPath())] = {'status': None, 'rigidbody': _rb}

        self.robots = init_robots(self.runtime, self._scene)
        self.objects = {}
        for obj in self.runtime.objects:
            _object = create_object(obj)
            _object.set_up_scene(self._scene)
            self.objects[obj.name] = _object
        self.loaded = True

    def clear_rigid_bodies(self):
        for rigid_body_name in self.scene_rigid_bodies.keys():
            if self.scene.object_exists(rigid_body_name):
                self.scene.remove_object(name=rigid_body_name)

    def save_info(self):
        """
        Saves the robot information and rigidbody statuses.
        """
        self.save_robot_info()
        self._save_rigidbody_statuses()

    def _save_rigidbody_statuses(self):
        """
        Saves the current status of all rigid bodies in the scene by querying their physics properties excluding
        those in the robot.

        Note:
            rigid prims within articulations aren't included since those RigidPrims' physical
            status (transform, velocity, etc) can't be set individually.
        """
        for rigid_body_name, rigid_body in self.scene_rigid_bodies.items():
            if not self.scene.object_exists(rigid_body_name):
                log.error(f'[cache_info] {rigid_body_name} does not exist.')
                continue
            rigid_body['status'] = get_rigidbody_status(rigid_body['rigidbody'])

    def _restore_rigidbody_statuses(self):
        """
        Restores the statuses of all rigid bodies in the scene based on their stored status data excluding
        those in the robot.
        """
        for rigid_body_name, rigid_body in self.scene_rigid_bodies.items():
            if rigid_body['status'] is None or not self.scene.object_exists(rigid_body_name):
                continue
            _rigid_body = RigidPrim(rigid_body_name)
            rigid_body['rigidbody'] = _rigid_body
            set_rigidbody_status(_rigid_body, rigid_body['status'])

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
        to the metrics' name, and each value is the result of the metric calculation.

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

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """
        Called before stepping the physics simulation.

        Args:
            time_step_index (int): [description]
            simulation_time (float): [description]
        """
        self.steps += 1
        return

    def save_robot_info(self):
        """
        Saves information of all robots in the task instance.
        """
        for robot in self.robots.values():
            robot.save_robot_info()

    def restore_info(self):
        """
        Restores the information and statuses of rigid bodies and robots.
        """
        self._restore_rigidbody_statuses()
        for robot in self.robots.values():
            robot.restore_robot_info()

    def post_reset(self) -> None:
        """Calls while doing a .reset() on the world."""
        self.steps = 0
        for robot in self.robots.values():
            robot.post_reset()
        return

    def cleanup(self) -> None:
        """
        Used to clean up the resources loaded in the task.
        """
        for obj in self.objects.keys():
            # Using try here because we want to ignore all exceptions
            try:
                self.scene.remove_object(obj)
            finally:
                log.info('[cleanup] objs cleaned.')
        for robot in self.robots.values():
            # Using try here because we want to ignore all exceptions
            log.info(f'[cleanup] cleanup robot {robot.name}')
            try:
                robot.cleanup()
                self.scene.remove_object(robot.name)
            finally:
                log.info('[cleanup] robots cleaned.')

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
