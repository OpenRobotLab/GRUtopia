# How to Add Custom Sensor

> This tutorial will show you how to add a sensor for a robot

The implementation of the sensor does not rely on any dependencies.
It serves as an abstraction layer that can either encapsulate existing sensors within Isaac Sim or generate synthetic data outputs to emulate a robot's sensor behavior.

## 1. Create Sensor's Configuration
Let's assume we need a camera sensor to collect depth information.

Our camera requires some input parameters:
- Switch for turning it on.
- Camera size.

Create the custom sensor's configuration in `grutopia_extension/config/sensors/__init__.py`. The configuration will cover all parameters that sensor needs.
```Python
class DepthCameraCfg(SensorModel):
    # Fields from params.
    type: Optional[str] = 'DepthCamera'
    enable: Optional[bool] = True
    size: Optional[Tuple[int, int]] = None
```

## 2. Inherit from `grutopia.core.robot.sensor`

```Python
from typing import Dict

from omni.isaac.sensor import Camera as i_Camera

from grutopia.core.config.robot import RobotUserConfig
from grutopia.core.robot.robot import BaseRobot, Scene
from grutopia.core.config.robot import SensorModel
from grutopia.core.robot.sensor import BaseSensor
from grutopia.core.util import log


@BaseSensor.register('DepthCamera')
class DepthCamera(BaseSensor):
    """
    wrap of isaac sim's Camera class
    """
     def __init__(self, config: SensorModel, robot: BaseRobot, name: str = None, scene: Scene = None):
        super().__init__(config, robot, scene)
        self.name = name
        self._camera = self.create_camera()

    def __init__(self,
                 robot_user_config: RobotUserConfig,
                 sensor_config: SensorModel,
                 robot: BaseRobot,
                 name: str = None,
                 scene: Scene = None):
        super().__init__(robot_user_config, sensor_config, robot, name)
        self.param = None
        if self.robot_user_config.sensors is not None:
            self.param = [p for p in self.robot_user_config.sensors if p.name == self.name][0]
        self._camera = self.create_camera()

    def create_camera(self) -> i_Camera:
        size = (1280, 720)
        if self.config.size is not None:
            size = self.config.size

        prim_path = self.robot_user_config.prim_path + '/' + self.sensor_config.prim_path
        return i_Camera(prim_path=prim_path, resolution=size)

    def sensor_init(self) -> None:
         if self.config.enable:
            self._camera.initialize()
            self._camera.add_distance_to_image_plane_to_frame()

    def get_data(self) -> Dict:
         if self.config.enable:
            depth = self._camera.get_depth()
            return {'depth': depth}
        return {}
```

## 3. Sensor Usage Preview
Follow the steps outlined in [how to use robot](../tutorials/how-to-use-robot.md) to add the new sensor to a robot. Then you can use it as demonstrated below:
```python
...
while env.simulation_app.is_running():
    ...
    obs, _, _, _, _ = env.step(action=env_action)
    print(obs['camera']['depth'])
    ...
env.simulation_app.close()
```
