# How to Use Robot

> This tutorial will show you how to use an existing robot.

## Supported Robots

The directory [`grutopia_extension/robots/`](https://github.com/OpenRobotLab/GRUtopia/tree/main/grutopia_extension/robots) contains a list of all available robots:

```
grutopia_extension/
└── robots
    ├── aliengo.py
    ├── franka.py
    ├── g1.py
    ├── gr1.py
    ├── h1.py
    ├── h1_with_hand.py
    ├── mocap_controlled_franka.py
    └── npc.py
```

## Robot Configuration

It is important to note that both controller and sensor must be used with a robot. Only the controllers and sensors specified in the robot's configuration will be available for that robot in simulation.

All our pre-defined robot config classes are located in the [`grutopia_extension/configs/robots`](https://github.com/OpenRobotLab/GRUtopia/tree/main/grutopia_extension/robots) folder.

Let's take `H1Robot` for instance, the file [`grutopia_extension/configs/robots/h1.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia_extension/robots/h1.py) includes some ready-to-use controllers and sensors configurations for `H1Robot`, as well as the config class for H1Robot:

```python
...
move_by_speed_cfg = H1MoveBySpeedControllerCfg(
   ...
)

move_to_point_cfg = MoveToPointBySpeedControllerCfg(
    ...
)

move_along_path_cfg = MoveAlongPathPointsControllerCfg(
   ...
)

rotate_cfg = RotateControllerCfg(
   ...
)

h1_camera_cfg = RepCameraCfg(name='camera', prim_path='logo_link/Camera', size=(640, 480))

h1_tp_camera_cfg = RepCameraCfg(name='tp_camera', prim_path='torso_link/TPCamera', size=(640, 480))


class H1RobotCfg(RobotCfg):
    ...
```

These configurations can be used to create a robot instance in the simulation environment.

## How to Create a Robot in Simulation

The following code snippet illustrates how to assemble a robot configuration by adding controllers and sensors and how to integrate the robot configuration into a task.

```python
from grutopia_extension.configs.robots.h1 import (
    H1RobotCfg,
    h1_camera_cfg,
    move_along_path_cfg,
    move_by_speed_cfg,
    rotate_cfg,
)

h1_1 = H1RobotCfg(
    controllers=[
        move_by_speed_cfg,
        move_along_path_cfg,
        rotate_cfg,
    ],
    sensors=[h1_camera_cfg.update(name='camera', size=(320, 240), enable=True)],
)

config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False),
    task_config=SingleInferenceTaskCfg(
        episodes=[
            SingleInferenceEpisodeCfg(
                scene_asset_path=gm.ASSET_PATH + '/scenes/empty.usd',
                scene_scale=[0.01, 0.01, 0.01],
                robots=[h1_1],
            ),
        ],
    ),
)

sim_runtime = SimulatorRuntime(config_class=config, headless=headless, native=headless)

import_extensions()  # The robot class is registered here.

# Create the environment.
env = Env(sim_runtime)
obs, _ = env.reset()

...

while env.simulation_app.is_running():
    i += 1
    env_action = {move_by_speed_cfg.name: [1.0, 0.0, 0.0]}  # Use move_by_speed controller
    obs, _, terminated, _, _ = env.step(action=env_action)

env.simulation_app.close()
```

Please read [`GRUtopia/grutopia/demo/h1_locomotion.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia/demo/h1_locomotion.py) for complete demo where a robot moves at a specified speed and direction.
