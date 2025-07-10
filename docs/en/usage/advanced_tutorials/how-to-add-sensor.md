# How to Add Custom Sensor

> This tutorial will show you how to add a sensor for a robot

The implementation of the sensor does not rely on any dependencies.
It serves as an abstraction layer that can either encapsulate native sensors within Isaac Sim or generate synthetic data outputs to emulate a robot's sensor behavior.


To add a custom sensor, you need to:
- Create a config class for sensor config, inheriting from the `grutopia.core.config.robot.SensorCfg`.
- Create a class for sensor, inheriting from the `grutopia.core.sensor.sensor.BaseSensor`.

## Create Config Class

Let's assume we need a depth camera.

The camera accepts the following config parameters:

- Flag to enable/disable the sensor.
- Camera resolution.

Here's an example of a config class for the sensor:

```Python
class DepthCameraCfg(SensorCfg):

    type: Optional[str] = 'DepthCamera'
    enable: Optional[bool] = True
    resolution: Optional[Tuple[int, int]] = None
```

Generally, when creating a new config class, reasonable default values for required fields should be specified, and sensor specific config fields can be added when necessary.

## Create Sensor Class

In the simplest scenario, the following methods are required to be implemented in your sensor class:

```python
from grutopia.core.robot.robot import BaseRobot, Scene
from grutopia.core.sensor.sensor import BaseSensor


@BaseSensor.register('DepthCamera')
class DepthCamera(BaseSensor):
    def __init__(self, config: DepthCameraCfg, robot: BaseRobot, scene: Scene):
        """Initialize the sensor with the given config.

        Args:
            config (DepthCameraCfg): sensor configuration.
            robot (BaseRobot): robot owning the sensor.
            scene (Scene): scene from isaac sim.
        """

    def get_data(self) -> Dict:
        """Get data from sensor.

        Returns:
            Dict: data dict of sensor.
        """
```

The `get_data` method gets the sensor data in each step.

For complete list of sensor methods, please refer to the [Sensor API documentation](../../api/robot.rst#module-grutopia.core.robot.sensor).

Please note that the registration of the sensor class is done through the `@BaseSensor.register` decorator, and the registered name should match the value of `type` field within the corresponding sensor config class (here is `DepthCamera`).

For some native sensor types, initialization after env reset is required for the sensor to work properly. The `post_reset` method of sensor is meant to be used in this situation.

An example of sensor class implementation is shown as following:

```python
from typing import Dict

from omni.isaac.sensor import Camera as i_Camera

from grutopia.core.robot.robot import BaseRobot, Scene
from grutopia.core.sensor.sensor import BaseSensor
from grutopia.core.util import log
from grutopia_extension.configs.sensors import DepthCameraCfg


@BaseSensor.register('DepthCamera')
class DepthCamera(BaseSensor):
    """
    wrap of isaac sim's Camera class
    """
     def __init__(self, config: DepthCameraCfg, robot: BaseRobot, name: str = None, scene: Scene = None):
        super().__init__(config, robot, scene)
        self.name = name
        self._camera = self.create_camera()

    def __init__(self,
                 config: DepthCameraCfg,
                 robot: BaseRobot,
                 name: str = None,
                 scene: Scene = None):
        super().__init__(config, robot, scene)

    def post_reset(self):
        if self.config.enable:
            resolution = (1280, 720) if self.config.resolution is None else self.config.resolution
            prim_path = self._robot.config.prim_path + '/' + self.config.prim_path
            self._camera = i_Camera(prim_path=prim_path, resolution=resolution)
            self._camera.initialize()

    def get_data(self) -> Dict:
         if self.config.enable:
            depth = self._camera.get_depth()
            return {'depth': depth}
        return {}
```
