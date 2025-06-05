from collections import OrderedDict
from functools import wraps
from typing import Any, Dict, List, Tuple, Union

import numpy as np
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.robots.robot import Robot as IsaacRobot
from omni.isaac.core.scenes import Scene
from pxr import Usd

from grutopia.core.config.robot import RobotCfg
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.util import log
from grutopia.core.util.physics_status_util import (
    get_articulation_status,
    set_articulation_status,
)


class BaseRobot:
    """Base class of robot."""

    robots = {}

    def __init__(self, config: RobotCfg, scene: Scene):
        self.name = config.name
        self.config = config
        self.isaac_robot: IsaacRobot | None = None
        self.controllers = {}
        self.sensors = {}
        self._scene = scene
        self.obs_keys = []
        self.articulation_status: Union[Dict[str, Any], None] = None
        self._rigid_body_map = {}

    def set_up_to_scene(self, scene: Scene):
        """Set up robot in the scene.

        Args:
            scene (Scene): scene to set up.
        """
        self.create_rigid_bodies()
        self._set_rigid_bodies()
        self._scene = scene
        robot_cfg = self.config
        if self.isaac_robot:
            scene.add(self.isaac_robot)
        for rigid_body in self.get_rigid_bodies():
            scene.add(rigid_body)
        from grutopia.core.robot.controller import BaseController, create_controllers
        from grutopia.core.robot.sensor import BaseSensor, create_sensors

        self.controllers: OrderedDict[str, BaseController] = create_controllers(robot_cfg, self, scene)
        self.sensors: OrderedDict[str, BaseSensor] = create_sensors(robot_cfg, self, scene)

    def clean_stale_rigid_bodies(self):
        """
        Removes stale rigid bodies from the scene.
        """
        if self._rigid_body_map:
            for rb in self._rigid_body_map.values():
                if self._scene.object_exists(rb.name):
                    self._scene.remove_object(rb.name)
                    log.debug(f'[clean_stale_rigid_body] stale rigid body {rb.name} removed')

    def create_rigid_bodies(self):
        """
        Create rigid bodies.
        """
        for prim in Usd.PrimRange.AllPrims(self.isaac_robot.prim):
            if prim.GetAttribute('physics:rigidBodyEnabled'):
                log.debug(f'[create_rigid_bodies] found rigid body at path: {prim.GetPath()}')
                _rb = RigidPrim(str(prim.GetPath()), name=str(prim.GetPath()))
                self._rigid_body_map[str(prim.GetPath())] = _rb

    def save_robot_info(self):
        """
        Saves the current state of the robot's articulation information.
        """
        log.info('saving robot info')
        self.articulation_status = get_articulation_status(self.isaac_robot)

    def restore_articulation(self, robot_status: Dict[str, Any] = None):
        """
        Restores the articulation of the robot based on the provided status or the internal state.
        """
        log.info('restoring articulation')
        if robot_status is not None:
            log.info('[restore_articulation] use external articulation status')
            self.articulation_status = robot_status
        if self.articulation_status:
            set_articulation_status(self.isaac_robot, self.articulation_status)

    def restore_robot_info(self):
        """
        Restores the robot's information and its state within the simulation environment.
        """
        if self.articulation_status is not None:
            self.isaac_robot._articulation_view._is_initialized = False
            self.isaac_robot._articulation_view._on_physics_ready('reset')
            self.restore_articulation()
            self.clean_stale_rigid_bodies()
            self.create_rigid_bodies()

        for sensor in self.sensors.values():
            sensor.restore_sensor_info()

    def post_reset(self):
        """Set up things after the env resets."""
        for sensor in self.sensors.values():
            sensor.post_reset()

    def cleanup(self):
        """
        Cleans up resources associated with the current instance by performing cleanup
        operations on controllers, sensors, and rigid bodies. Ensures that all associated
        objects are properly removed from the scene.
        """
        for controller in self.controllers.values():
            controller.cleanup()
        for sensor in self.sensors.values():
            sensor.cleanup()
        for rigid_body in self.get_rigid_bodies():
            try:
                if self._scene.object_exists(rigid_body.name):
                    self._scene.remove_object(rigid_body.name)
                    log.debug(f'[cleanup] rigid body {rigid_body.name} removed from scene')
            except RuntimeError as e:
                log.error(e)
                continue
        log.debug(f'[cleanup] robot {self.name} clean up')

    def apply_action(self, action: dict):
        """Apply actions of controllers to robot.

        Args:
            action (dict): action dict.
              key: controller name.
              value: corresponding action array.
        """
        raise NotImplementedError()

    def get_obs(self) -> OrderedDict:
        """Get observation of robot, including controllers, sensors, and world pose.

        Raises:
            NotImplementedError: _description_
        """
        raise NotImplementedError()

    def _get_controllers_and_sensors_obs(self) -> Tuple[OrderedDict[str, Any], OrderedDict[str, Any]]:
        """
        Retrieves the observations from all controllers and sensors, returning them as two separate ordered dictionaries.

        Returns:
            Tuple[OrderedDict[str, Any], OrderedDict[str, Any]]: A tuple containing two ordered dictionaries. The first
            dictionary contains the observations from the controllers, and the second contains the data from the sensors.
        """
        controllers_obs = OrderedDict()
        sensors_obs = OrderedDict()
        for c_obs_name, controller_obs in self.controllers.items():
            controllers_obs[c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            sensors_obs[sensor_name] = sensor_obs.get_data()
        return controllers_obs, sensors_obs

    def get_robot_base(self) -> RigidPrim:
        """
        Get the base link of this robot.

        Returns:
            RigidPrim: rigid prim of the robot base link.
        """
        raise NotImplementedError()

    def get_robot_scale(self) -> np.ndarray:
        """Get robot scale.

        Returns:
            np.ndarray: robot scale in (x, y, z).
        """
        return self.isaac_robot.get_local_scale()

    def get_robot_articulation(self) -> IsaacRobot:
        """Get isaac robots instance (articulation).

        Returns:
            Robot: robot articulation.
        """
        return self.isaac_robot

    def get_controllers(self):
        return self.controllers

    def _set_rigid_bodies(self):
        pass

    def get_rigid_bodies(self) -> List[RigidPrim]:
        return []

    def _make_ordered(self, obs: Dict = None) -> OrderedDict:
        if obs is None:
            return OrderedDict()
        if not self.obs_keys:
            self.obs_keys = [i for i in obs.keys()]
        return OrderedDict((key, obs[key]) for key in self.obs_keys)

    @classmethod
    def register(cls, name: str):
        """Register a robot class with its name(decorator).

        Args:
            name(str): name of the robot class.
        """

        def decorator(robot_class):
            cls.robots[name] = robot_class

            @wraps(robot_class)
            def wrapped_function(*args, **kwargs):
                return robot_class(*args, **kwargs)

            return wrapped_function

        return decorator


def _remove_suffix(name: str) -> str:
    """Remove the suffix after the last underscore in the name, if exists."""
    last_underscore_index = name.rfind('_')
    if last_underscore_index > 0 and name[last_underscore_index + 1 :].isdigit():
        return name[:last_underscore_index]
    return name


def create_robots(runtime: TaskRuntime, scene: Scene) -> OrderedDict[str, BaseRobot]:
    """Create robot instances in runtime.

    Args:
        runtime (TaskRuntime): task runtime.
        scene (Scene): isaac scene.

    Returns:
        OrderedDict[str, BaseRobot]: robot instances dictionary.
    """
    robot_map = OrderedDict()
    for robot in runtime.robots:
        if robot.type not in BaseRobot.robots:
            raise KeyError(f'[create_robots] unknown robot type "{robot.type}"')
        robot_cls = BaseRobot.robots[robot.type]
        robot_ins: BaseRobot = robot_cls(robot, scene)
        robot_map[_remove_suffix(robot.name)] = robot_ins
        robot_ins.set_up_to_scene(scene)
        log.debug(f'[create_robots] {robot.name} loaded')
    return robot_map
