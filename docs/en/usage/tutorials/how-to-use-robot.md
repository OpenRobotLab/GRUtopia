# How to Use Robot

> This tutorial guides you on how to create a robot in an episode.

## Pre-defined Robots

The directory [`grutopia_extension/robots/`](https://github.com/OpenRobotLab/GRUtopia/tree/main/grutopia_extension/robots) contains a list of all pre-defined robots:

```
grutopia_extension/
└── robots
    ├── aliengo.py
    ├── franka.py
    ├── g1.py
    ├── gr1.py
    ├── h1.py
    ├── h1_with_hand.py
    ├── jetbot.py
    ├── mocap_controlled_franka.py
    └── npc.py
    ...
```

## Robot Config Class

Each robot has its own config class, and we use the config class to create a robot in simulation.

All our pre-defined robot config classes are located in the [`grutopia_extension/configs/robots`](https://github.com/OpenRobotLab/GRUtopia/tree/main/grutopia_extension/configs/robots) folder.

Let's take `JetbotRobot` for instance, the file [`grutopia_extension/configs/robots/jetbot.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia_extension/robots/jetbot.py) includes the config class for JetbotRobot:

```python
class JetbotRobotCfg(RobotCfg):
    # meta info
    name: Optional[str] = 'jetbot'
    type: Optional[str] = 'JetbotRobot'
    prim_path: Optional[str] = '/World/jetbot'
    create_robot: Optional[bool] = True
    usd_path: Optional[str] = gm.ASSET_PATH + '/robots/jetbot/jetbot.usd'
```

These configurations can be used to create a robot instance in the simulation environment.

## How to Create a Robot

The following code illustrates how to add a robot config to the episode config to create the robot in that episode.

```{code-block} python
:emphasize-lines: 7-9,23-28,50

from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util import has_display
from grutopia.macros import gm
from grutopia_extension import import_extensions
from grutopia_extension.configs.robots.jetbot import (
    JetbotRobotCfg,
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
    obs, _, terminated, _, _ = env.step(action={})

    if i % 1000 == 0:
        print(i)
        print(obs)

env.simulation_app.close()
```

<video width="720" height="405" controls>
    <source src="../../_static/video/tutorial_use_robot.webm" type="video/webm">
</video>

In the above example, we create a jetbot in the first episode with initial position at `(0.0, 0.0, 0.0)` and scale 5.0. And in each env step all available observations of that robot are included in the `obs` returned by `env.step`:

```
{
  'position': array([-0.02586134, -0.01573028,  0.16749829], dtype=float32),
  'orientation': array([ 9.9999404e-01, -4.3268519e-06, -3.2296611e-03, -1.3232718e-03],
      dtype=float32),
  'joint_positions': array([ 3.2044773e-05, -5.2124355e-04], dtype=float32),
  'joint_velocities': array([0.03243133, 0.03371198], dtype=float32),
  'controllers': {},
  'sensors': {},
  'render': False,
}
```

The obs returned are defined in the `get_obs` method of the robot. For jetbot, you can check it in [`grutopia_extension/robots/jetbot.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia_extension/robots/jetbot.py):

```python
class JetbotRobot(BaseRobot):
    def get_obs(self):
        position, orientation = self.isaac_robot.get_world_pose()

        # custom
        obs = {
            'position': position,
            'orientation': orientation,
            'joint_positions': self.isaac_robot.get_joint_positions(),
            'joint_velocities': self.isaac_robot.get_joint_velocities(),
            'controllers': {},
            'sensors': {},
        }

        # common
        for c_obs_name, controller_obs in self.controllers.items():
            obs['controllers'][c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            obs['sensors'][sensor_name] = sensor_obs.get_data()
        return obs
```
