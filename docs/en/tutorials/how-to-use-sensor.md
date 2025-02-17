# How to Use Sensor

> This tutorial will show you how to use existing sensors.

## Supported Sensors

In `grutopia_extension/config/sensors/__init__.py`, we can observe all available sensors:

```Python
class CameraCfg(SensorModel):
   ...


class RepCameraCfg(SensorModel):
    ...


class MocapControlledCameraCfg(SensorModel):
    ...
```

## How to Use a Sensor

In the main loop of the script, we can obtain observations collected from a sensor. In the example below, we retrieve the RGBA data from the camera.
```Python
...
path = [(1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (3.0, 4.0, 0.0)]
move_action = {move_along_path_cfg.name: [path]}

while env.simulation_app.is_running():
    env_action = move_action
    obs, _, _, _, _ = env.step(actions=env_actions)
    rgba = obs["camera"]["rgba"]
    ...
env.simulation_app.close()
```

We can see that `step()` method returns obs as a dictionary, and the data collected by the corresponding sensor can be retrieved by accessing the values in the dictionary.


It is important to specify the sensors in the robot before use. Please refer to the [how to use robot](./how-to-use-robot.md) for more detailed information.
