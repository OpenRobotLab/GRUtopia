from collections import OrderedDict

import numpy as np
from omni.isaac.core.scenes import Scene
from omni.isaac.wheeled_robots.robots import WheeledRobot

from grutopia.core.robot.robot import BaseRobot
from grutopia.core.util import log
from grutopia_extension.configs.robots.jetbot import JetbotRobotCfg


@BaseRobot.register('JetbotRobot')
class JetbotRobot(BaseRobot):
    def __init__(self, config: JetbotRobotCfg, scene: Scene):
        super().__init__(config, scene)
        self._sensor_config = config.sensors
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'jetbot {config.name} position    : ' + str(self._start_position))
        log.debug(f'jetbot {config.name} orientation : ' + str(self._start_orientation))

        usd_path = config.usd_path

        log.debug(f'jetbot {config.name} usd_path         : ' + str(usd_path))
        log.debug(f'jetbot {config.name} config.prim_path : ' + str(config.prim_path))
        self.prim_path = str(config.prim_path)
        self.isaac_robot = WheeledRobot(
            prim_path=config.prim_path,
            name=config.name,
            wheel_dof_names=['left_wheel_joint', 'right_wheel_joint'],
            create_robot=True,
            position=self._start_position,
            orientation=self._start_orientation,
            usd_path=usd_path,
        )

        self._robot_scale = np.array([1.0, 1.0, 1.0])
        if config.scale is not None:
            self._robot_scale = np.array(config.scale)
            self.isaac_robot.set_local_scale(self._robot_scale)
        self.obs_keys = ['position', 'orientation', 'joint_positions', 'joint_velocities', 'controllers', 'sensors']

    def get_robot_scale(self):
        return self._robot_scale

    def get_world_pose(self):
        return self.isaac_robot.get_world_pose()

    def apply_action(self, action: dict):
        """
        Args:
            action (dict): inputs for controllers.
        """
        for controller_name, controller_action in action.items():
            # print(controller_name, "=============", controller_action)
            if controller_name not in self.controllers:
                log.warning(f'unknown controller {controller_name} in action')
                continue
            controller = self.controllers[controller_name]
            control = controller.action_to_control(controller_action)
            self.isaac_robot.apply_action(control)

    def get_obs(self) -> OrderedDict:
        position, orientation = self.isaac_robot.get_world_pose()

        # custom
        obs = {
            'position': position,
            'orientation': orientation,
            'joint_positions': self.isaac_robot.get_joint_positions(),
            'joint_velocities': self.isaac_robot.get_joint_velocities(),
            'controllers': {},
            'sensors': {},
        }

        # common
        for c_obs_name, controller_obs in self.controllers.items():
            obs['controllers'][c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            obs['sensors'][sensor_name] = sensor_obs.get_data()
        return OrderedDict((key, obs[key]) for key in self.obs_keys)
