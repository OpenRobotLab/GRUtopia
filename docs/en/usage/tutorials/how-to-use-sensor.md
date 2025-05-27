# How to Use Sensor

> This tutorial guides you on how to use a sensor to retrieve corresponding observation data.

## Pre-defined Sensors

The directory [`grutopia_extension/sensors/`](https://github.com/OpenRobotLab/GRUtopia/tree/main/grutopia_extension/sensors) contains a list of all available sensors::

```
grutopia_extension/
└── sensors
    ├── camera.py
    ├── mocap_controlled_camera.py
    └── rep_camera.py
    ...
```

For each robot, we provide some ready-to-use sensor configurations for each robot in `grutopia_extension/configs/robots/{robot_name}.py`.

## How to Use a Sensor

Typically, a sensor should be used with a robot. So first of all, the sensor configuration should be added to the sensor list in robot configuration:

```{code-block} python
:emphasize-lines: 10,30,55-56

from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util import has_display
from grutopia.macros import gm
from grutopia_extension import import_extensions
from grutopia_extension.configs.robots.jetbot import (
    JetbotRobotCfg,
    move_by_speed_cfg,
    camera_cfg,
)
from grutopia_extension.configs.tasks import (
    SingleInferenceEpisodeCfg,
    SingleInferenceTaskCfg,
)

headless = not has_display()

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False),
    task_config=SingleInferenceTaskCfg(
        episodes=[
            SingleInferenceEpisodeCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                robots=[
                    JetbotRobotCfg(
                        position=(0.0, 0.0, 0.0),
                        scale=(5.0, 5.0, 5.0),
                        controllers=[move_by_speed_cfg],
                        sensors=[camera_cfg],
                    )
                ],
            ),
        ],
    ),
)

sim_runtime = SimulatorRuntime(config_class=config, headless=headless, native=headless)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
obs, _ = env.reset()

i = 0

while env.simulation_app.is_running():
    i += 1
    action = {move_by_speed_cfg.name: [0.5, 0.5]}
    obs, _, terminated, _, _ = env.step(action=action)

    if i % 1000 == 0:
        print(i)
        for k, v in obs['sensors'][camera_cfg.name].items():
            print(f'key: {k}, value: {v}')

env.simulation_app.close()
```

<video width="720" height="405" controls>
    <source src="../../_static/video/tutorial_use_sensor.webm" type="video/webm">
</video>

In the above example, first we import the `move_by_speed_cfg` for jetbot. It'a a ready-to-use sensor config for jetbot to use the `Camera` to get observations:

[`grutopia_extension/configs/robots/jetbot.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia_extension/configs/robots/jetbot.py)

```python
camera_cfg = CameraCfg(
    name='camera',
    prim_path='chassis/rgb_camera/jetbot_camera',
    resolution=(640, 360),
)
```

The sensor config is then added to the robot config to declare it as an available sensor for the robot in that episode. In each step, we can read the observations from the `obs` dict returned by `env.step()`. Observations from certain sensor are stored in `obs['sensors'][{sensor_name}]`. The data structure of observation is defined by the `get_data` method of the specific sensor. For the above example, we can check it in the `Camera` class:

[`grutopia_extension/sensors/camera.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia_extension/sensors/camera.py)

```python
class Camera(BaseSensor):
    def get_data(self) -> Dict:
        if self.config.enable:
            rgba = self._camera.get_rgba()
            frame = self._camera.get_current_frame()
            return {'rgba': rgba, 'frame': frame}
        return {}
```

So the rgba and frame data would be printed every 1000 steps in our example.
