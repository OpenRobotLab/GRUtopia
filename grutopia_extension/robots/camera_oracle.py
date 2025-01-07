import os

import numpy as np
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.rotations import euler_angles_to_quat

from grutopia.core.config.robot import RobotUserConfig as Config
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.robot.robot_model import RobotModel
from grutopia.core.util import log
from grutopia_extension.robots.humanoid import Humanoid


@BaseRobot.register('CameraRobot')
class CameraRobot(BaseRobot):

    def __init__(self, config: Config, robot_model: RobotModel, scene: Scene):
        super().__init__(config, robot_model, scene)
        self._sensor_config = robot_model.sensors
        self._gains = robot_model.gains
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'humanoid {config.name}: position    : ' + str(self._start_position))
        log.debug(f'humanoid {config.name}: orientation : ' + str(self._start_orientation))

        usd_path = robot_model.usd_path

        log.debug(f'humanoid {config.name}: usd_path         : ' + str(usd_path))
        log.debug(f'humanoid {config.name}: config.prim_path : ' + str(config.prim_path))
        self.isaac_robot = Humanoid(
            prim_path=config.prim_path,
            name=config.name,
            position=self._start_position,
            orientation=self._start_orientation,
            usd_path=usd_path,
        )

        self._robot_scale = np.array([1.0, 1.0, 1.0])
        self.size = (512, 512)
        self.camera_prim_path = self.user_config.prim_path + '/' + self._sensor_config[0].prim_path
        self.camera_prim = XFormPrim(os.path.dirname(self.camera_prim_path))

    def get_ankle_height(self):
        return None

    def get_robot_scale(self):
        return self._robot_scale

    def get_robot_base(self):
        return None

    def get_robot_ik_base(self):
        return None

    def get_world_pose(self):
        # rp = rep.create.render_product(self.camera_prim_path, self.size)
        # camera_params = rep.annotators.get("CameraParams").attach(rp)
        # camera_transform = camera_params.get_data()['cameraViewTransform'].reshape(4, 4)
        # try:
        #     camera_transform = np.linalg.inv(camera_transform)
        #     camera_position = camera_transform[3, :3]
        #     camera_orientation = matrix_to_euler_angles(camera_transform[:3, :3].T)
        #     return camera_position, camera_orientation
        # except:
        return self.camera_prim.get_world_pose()

    def apply_action(self, action: dict):
        """
        Args:
            action (dict): inputs for controllers.
        """
        for controller_name, controller_action in action.items():
            if controller_name not in self.controllers:
                log.warn(f'unknown controller {controller_name} in action')
                continue
            controller = self.controllers[controller_name]
            control = controller.action_to_control(controller_action)
            rot = euler_angles_to_quat(control['rot']) if control['rot'] is not None else None
            self.camera_prim.set_world_pose(control['pos'], rot)
            # with rep.get.camera(self.camera_prim_path):
            #     rep.modify.pose(position=control['pos'], rotation=control['rot'], rotation_order='XYZ')

    def get_obs(self):
        position, orientation = self.get_world_pose()

        # custom
        obs = {
            'position': position,
            'orientation': orientation,
        }

        # common
        for c_obs_name, controller_obs in self.controllers.items():
            obs[c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            obs[sensor_name] = sensor_obs.get_data()
        return obs
