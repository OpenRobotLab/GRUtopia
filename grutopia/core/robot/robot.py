from functools import wraps
from typing import Dict, List

import numpy as np
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.robots.robot import Robot as IsaacRobot
from omni.isaac.core.scenes import Scene

from grutopia.core.config.robot import RobotCfg
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.util import log


class BaseRobot:
    """Base class of robot."""

    robots = {}

    def __init__(self, config: RobotCfg, scene: Scene):
        self.name = config.name
        self.config = config
        self.isaac_robot: IsaacRobot | None = None
        self.controllers = {}
        self.sensors = {}
        self._scene = None

    def set_up_to_scene(self, scene: Scene):
        """Set up robot in the scene.

        Args:
            scene (Scene): scene to set up.
        """
        self._scene = scene
        robot_cfg = self.config
        if self.isaac_robot:
            scene.add(self.isaac_robot)
            log.debug('self.isaac_robot: ' + str(self.isaac_robot))
        for rigid_body in self.get_rigid_bodies():
            scene.add(rigid_body)
        from grutopia.core.robot.controller import BaseController, create_controllers
        from grutopia.core.robot.sensor import BaseSensor, create_sensors

        self.controllers: Dict[str, BaseController] = create_controllers(robot_cfg, self, scene)
        self.sensors: Dict[str, BaseSensor] = create_sensors(robot_cfg, self, scene)

    def post_reset(self):
        """Set up things that happen after the world resets."""
        for sensor in self.sensors.values():
            sensor.post_reset()

    def cleanup(self):
        for controller in self.controllers.values():
            controller.cleanup()
        for sensor in self.sensors.values():
            sensor.cleanup()
        for rigid_body in self.get_rigid_bodies():
            self._scene.remove_object(rigid_body.name)
            log.debug(f'rigid body {rigid_body} removed')
        log.debug(f'robot {self.name} clean up')

    def apply_action(self, action: dict):
        """Apply actions of controllers to robot.

        Args:
            action (dict): action dict.
              key: controller name.
              value: corresponding action array.
        """
        raise NotImplementedError()

    def get_obs(self) -> dict:
        """Get observation of robot, including controllers, sensors, and world pose.

        Raises:
            NotImplementedError: _description_
        """
        raise NotImplementedError()

    def get_robot_base(self) -> RigidPrim:
        """
        Get base link of robot.

        Returns:
            RigidPrim: rigid prim of robot base link.
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

    def get_rigid_bodies(self) -> List[RigidPrim]:
        return []

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


def create_robots(runtime: TaskRuntime, scene: Scene) -> Dict[str, BaseRobot]:
    """Create robot instances in runtime.

    Args:
        runtime (TaskRuntime): task runtime.
        scene (Scene): isaac scene.

    Returns:
        Dict[str, BaseRobot]: robot instances dictionary.
    """
    robot_map = {}
    for robot in runtime.robots:
        if robot.type not in BaseRobot.robots:
            raise KeyError(f'unknown robot type "{robot.type}"')
        robot_cls = BaseRobot.robots[robot.type]
        robot_ins: BaseRobot = robot_cls(robot, scene)
        robot_map[robot.name] = robot_ins
        robot_ins.set_up_to_scene(scene)
        log.debug(f'===== {robot.name} loaded =====')
    return robot_map
