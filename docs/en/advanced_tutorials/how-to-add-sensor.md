# How to add custom sensor

> This tutorial will show you how to add a sensor for a robot

Before this tutorial, you should read:
- [how to add robot](./how-to-add-robot.md)

## 1. Create with `grutopia.core.robot.sensor`

The sensors in `grutopia` are not just tensor. They are interfaces for robots to passively receive all kinds of
information.

The only thing we should matter is: implement `BaseSensor` from `grutopia.core.robot.sensor`

Camera sensor FYI

```Python
from typing import Dict

from omni.isaac.sensor import Camera as i_Camera

from grutopia.core.config.robot import RobotUserConfig
from grutopia.core.robot.robot import BaseRobot, Scene
from grutopia.core.robot.robot_model import SensorModel
from grutopia.core.robot.sensor import BaseSensor
from grutopia.core.util import log


@BaseSensor.register('Camera')
class Camera(BaseSensor):
    """
    wrap of isaac sim's Camera class
    """

    def __init__(self,
                 robot_user_config: RobotUserConfig,
                 sensor_config: SensorModel,
                 robot: BaseRobot,
                 name: str = None,
                 scene: Scene = None):
        super().__init__(robot_user_config, sensor_config, robot, name)
        self.param = None
        if self.robot_user_config.sensor_params is not None:
            self.param = [p for p in self.robot_user_config.sensor_params if p.name == self.name][0]
        self._camera = self.create_camera()

    def create_camera(self) -> i_Camera:
        size = (1280, 720)
        if self.param is not None:
            size = self.param.size

        prim_path = self.robot_user_config.prim_path + '/' + self.sensor_config.prim_path
        log.debug('camera_prim_path: ' + prim_path)
        log.debug('name            : ' + '_'.join(
            [self.robot_user_config.name, self.sensor_config.name]))
        return i_Camera(prim_path=prim_path, resolution=size)

    def sensor_init(self) -> None:
        if self.param is not None:
            if self.param.switch:
                self._camera.initialize()
                self._camera.add_distance_to_image_plane_to_frame()
                self._camera.add_semantic_segmentation_to_frame()
                self._camera.add_instance_segmentation_to_frame()
                self._camera.add_instance_id_segmentation_to_frame()
                self._camera.add_bounding_box_2d_tight_to_frame()

    def get_data(self) -> Dict:
        if self.param is not None:
            if self.param.switch:
                rgba = self._camera.get_rgba()
                depth = self._camera.get_depth()
                frame = self._camera.get_current_frame()
                return {'rgba': rgba, 'depth': depth, 'frame': frame}
        return {}
```

## 2. Register at `robot_models`

Add sensor for robots in `grutopia_extension/robots/robot_models.yaml`.

```yaml
robots:
  - type: "HumanoidRobot"
    ...
    sensors:
      - name: "camera"
        prim_path: "relative/prim/path/to/camera"  # relative path
        type: "Camera"  # map to key in `register`
```

## 3. Write a demo

In simulation_app's step loop:

```Python
   ...
   obs = env.step(actions)
   photo = obs['robot_name_in_config']['camera']['frame']['rgba']  # `camera` is sensor name in model
   ...
```
