# How to Use Sensor

> This tutorial will show you how to use existing sensors.

## Supported Sensors

The directory [`grutopia_extension/sensors/`](https://github.com/OpenRobotLab/GRUtopia/tree/main/grutopia_extension/sensors) contains a list of all available sensors::

```
grutopia_extension/
└── sensors
    ├── camera.py
    ├── mocap_controlled_camera.py
    └── rep_camera.py
```

For each robot, we provide some ready-to-use sensor configurations in `grutopia_extension/configs/robots/{robot_name}.py`.

## How to Use a Sensor

Typically, a sensor should be used with a robot. So first of all, the sensor configuration should be added to the sensor list in robot configuration:

```python
camera_cfg = CameraCfg(...)

h1_1 = H1RobotCfg(
    position=(0.0, 0.0, 1.05),
    sensors=[
        camera_cfg,
    ],
    ...
）
```

In the main loop of the script, we can obtain observations collected from a sensor. In the example below, we retrieve the RGBA data from the camera.

```python
...

while env.simulation_app.is_running():
    env_action = {...}
    obs, _, _, _, _ = env.step(actions=env_actions)
    rgba = obs['sensors']["camera"]["rgba"]
    ...
env.simulation_app.close()
```

We can see that `step()` method returns obs as a dictionary, and the data collected by the corresponding sensor can be retrieved by accessing the values in the dictionary.


It is important to specify the sensors in the robot before use. Please refer to the [how to use robot](./how-to-use-robot.md) for more detailed information.
