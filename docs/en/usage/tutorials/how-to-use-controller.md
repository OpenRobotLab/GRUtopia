# How to Use Controller

> This tutorial guides you on how to use a controller to control a robot.

## Pre-defined Controllers

The directory [`grutopia_extension/controllers/`](https://github.com/OpenRobotLab/GRUtopia/tree/main/grutopia_extension/controllers) contains a list of all pre-defined controllers:

```
grutopia_extension/
└── controllers
    ├── aliengo_move_by_speed_controller.py
    ├── dd_controller.py
    ├── franka_mocap_teleop_controller.py
    ├── g1_move_by_speed_controller.py
    ├── gr1_move_by_speed_controller.py
    ├── gr1_teleop_controller.py
    ├── gripper_controller.py
    ├── h1_move_by_speed_controller.py
    ├── ik_controller.py
    ├── joint_controller.py
    ├── move_along_path_points_controller.py
    ├── move_to_point_by_speed_controller.py
    ├── move_to_point_oracle_controller.py
    ├── recover_controller.py
    ├── rmpflow_controller.py
    └── rotate_controller.py
    ...
```

For each robot, we provide some ready-to-use controller configurations for each robot in `grutopia_extension/configs/robots/{robot_name}.py`.

## How to Use a Controller

A controller must be used with a robot, and the corresponding action must be specified in each step to control the robot with that controller.

```{code-block} python
:emphasize-lines: 9,28,48-49

from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util import has_display
from grutopia.macros import gm
from grutopia_extension import import_extensions
from grutopia_extension.configs.robots.jetbot import (
    JetbotRobotCfg,
    move_by_speed_cfg,
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
        print(obs)

env.simulation_app.close()
```

<video width="720" height="405" controls>
    <source src="../../_static/video/tutorial_use_controller.webm" type="video/webm">
</video>

In the above example, first we import the `move_by_speed_cfg` for jetbot. It'a a ready-to-use controller config for jetbot to use the `DifferentialDriveController` to move around:

[`grutopia_extension/configs/robots/jetbot.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia_extension/configs/robots/jetbot.py)

```python
move_by_speed_cfg = DifferentialDriveControllerCfg(name='move_by_speed', wheel_base=0.1125, wheel_radius=0.03)
```

The controller config is then added to the robot config to declare it as an available controller for the robot in that episode. In each step, we define the action to be applied to the robot, that is, a dict with the controller name as the key and desired action as the value. The format of action is defined by the `action_to_control` method of the specific controller. For the above example, we can check it in the `DifferentialDriveController` class:

[`grutopia_extension/controllers/dd_controller.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia_extension/controllers/dd_controller.py)

```python
class DifferentialDriveController(BaseController):
    def action_to_control(self, action: List | np.ndarray) -> ArticulationAction:
        """
        Args:
            action (List | np.ndarray): n-element 1d array containing:
              0. forward_speed (float)
              1. rotation_speed (float)
        """
        assert len(action) == 2, 'action must contain 2 elements'
        return self.forward(
            forward_speed=action[0],
            rotation_speed=action[1],
        )
```

So with the action `[0.5, 0.5]` the jetbot will move forward at 0.5 m/s and rotate at 0.5 rad/s.

You can try more controllers with the controller configurations defined in [`grutopia_extension/configs/robots/jetbot.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia_extension/configs/robots/jetbot.py).
