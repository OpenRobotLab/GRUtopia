import math
from typing import Dict

from omni.isaac.sensor import Camera as i_Camera

from grutopia.core.robot.robot import BaseRobot, Scene
from grutopia.core.robot.sensor import BaseSensor
from grutopia_extension.configs.sensors import LayoutEditMocapControlledCameraCfg
from grutopia_extension.sensors.mocap_controlled_camera import CameraMover

DISPLACEMENT_THRESHOLD = 0.05
NUM_SPECIFIC_POSE_FRAMES = 60


@BaseSensor.register('LayoutEditMocapControlledCamera')
class LayoutEditMocapControlledCamera(BaseSensor):
    """
    wrap of isaac sim's Camera class
    """

    def __init__(
        self, config: LayoutEditMocapControlledCameraCfg, robot: BaseRobot, name: str = None, scene: Scene = None
    ):
        super().__init__(config, robot, scene)
        self.name = name
        self.config = config

        self.layout_camera_mover = LayoutCameraMover([0, 0, 0])

    def create_camera(self) -> i_Camera:
        """Create an isaac-sim camera object.

        Initializes the camera's resolution and prim path based on configuration.

        Returns:
            i_Camera: The initialized camera object.
        """
        # Initialize the params for the camera
        resolution = self.config.resolution if self.config.resolution else (320, 240)
        translation = self.config.translation if self.config.translation else None
        orientation = self.config.orientation if self.config.translation else None

        prim_path = '/World' + '/' + self.config.prim_path
        return i_Camera(prim_path=prim_path, resolution=resolution, translation=translation, orientation=orientation)

    def post_reset(self):
        if self.config.enable:
            self._camera = self.create_camera()
            self._camera.initialize()
            self._camera.add_bounding_box_2d_tight_to_frame()

    def get_data(self) -> Dict:
        if self.config.enable:
            rgba = self._camera.get_rgba()
            frame = self._camera.get_current_frame()
            return {'rgba': rgba, 'frame': frame}
        return {}

    @property
    def camera(self):
        return self._camera


class LayoutCameraMover(CameraMover):
    def __init__(self, target_point, rate=[3, 5, 5]):
        super().__init__(target_point, rate)

    def condition_judgment(self, results):
        lh_bones_kps = results.get('lh_bones_kps', None)
        if lh_bones_kps is None:
            self.is_move = False
            return

        thumb_finger = lh_bones_kps[4]
        index_finger = lh_bones_kps[12]
        distance = math.sqrt(
            (index_finger[0] - thumb_finger[0]) ** 2
            + (index_finger[1] - thumb_finger[1]) ** 2
            + (index_finger[2] - thumb_finger[2]) ** 2
        )

        if distance < DISPLACEMENT_THRESHOLD:
            self.is_move = True
        else:
            self.is_move = False
