from functools import wraps
from typing import Dict, List, Tuple

import numpy as np
from omni.isaac.core.prims import RigidPrim
from omni.isaac.core.robots.robot import Robot as IsaacRobot
from omni.isaac.core.scenes import Scene

from grutopia.core.config.robot import RobotModel
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.util import log


class BaseRobot:
    """Base class of robot."""

    robots = {}

    def __init__(self, robot_model: RobotModel, scene: Scene):
        self.name = robot_model.name
        self.robot_model = robot_model
        self.isaac_robot: IsaacRobot | None = None
        self.controllers = {}
        self.sensors = {}

    def set_up_to_scene(self, scene: Scene):
        """Set up robot in the scene.

        Args:
            scene (Scene): scene to set up.
        """
        robot_model = self.robot_model
        if self.isaac_robot:
            scene.add(self.isaac_robot)
            log.debug('self.isaac_robot: ' + str(self.isaac_robot))
        for rigid_body in self.get_rigid_bodies():
            scene.add(rigid_body)
        from grutopia.core.robot.controller import BaseController, create_controllers
        from grutopia.core.robot.sensor import BaseSensor, create_sensors

        self.controllers: Dict[str, BaseController] = create_controllers(robot_model, self, scene)
        self.sensors: Dict[str, BaseSensor] = create_sensors(robot_model, self, scene)

    def post_reset(self):
        """Set up things that happen after the world resets."""
        for sensor in self.sensors.values():
            sensor.reset()

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

    def get_robot_ik_base(self) -> RigidPrim:
        """Get base link of ik controlled parts.

        Returns:
            RigidPrim: rigid prim of ik base link.
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

    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.isaac_robot.get_world_pose()

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
