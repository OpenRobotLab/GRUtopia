from typing import Dict

from omni.isaac.sensor import Camera as i_Camera

from grutopia.core.config.robot import SensorModel
from grutopia.core.robot.robot import BaseRobot, Scene
from grutopia.core.robot.sensor import BaseSensor
from grutopia.core.util import log


@BaseSensor.register('Camera')
class Camera(BaseSensor):
    """
    wrap of isaac sim's Camera class
    """

    def __init__(self, config: SensorModel, robot: BaseRobot, name: str = None, scene: Scene = None):
        super().__init__(config, robot, scene)
        self.name = name
        self._camera = self.create_camera()

    def create_camera(self) -> i_Camera:
        """Create an isaac-sim camera object.

        Initializes the camera's resolution and prim path based on configuration.

        Returns:
            i_Camera: The initialized camera object.
        """
        # Initialize the default resolution for the camera
        size = (320, 240)
        # Use the configured camera size if provided.
        if self.config.size is not None:
            size = self.config.size

        prim_path = self._robot.user_config.prim_path + '/' + self.config.prim_path
        log.debug('camera_prim_path: ' + prim_path)
        log.debug('name            : ' + self.config.name)
        log.debug(f'size            : {size}')
        return i_Camera(prim_path=prim_path, resolution=size)

    def init(self) -> None:
        """
        Initialize the camera sensor.

        TODO Add `camera frame config` into GRUtopia config file.
        """
        if self.config.enable:
            self._camera.initialize()
            # self._camera.add_pointcloud_to_frame()
            # self._camera.add_distance_to_image_plane_to_frame()
            # self._camera.add_semantic_segmentation_to_frame()
            # self._camera.add_instance_segmentation_to_frame()
            # self._camera.add_instance_id_segmentation_to_frame()
            self._camera.add_bounding_box_2d_tight_to_frame()

    def get_data(self) -> Dict:
        if self.config.enable:
            rgba = self._camera.get_rgba()
            frame = self._camera.get_current_frame()
            return {'rgba': rgba, 'frame': frame}
        return {}

    def reset(self):
        del self._camera
        self._camera = self.create_camera()
        self.init()
