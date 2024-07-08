# How to use robot

> This tutorial will show you how to use an existing robot.

## All available robots

See `grutopia_extension/robots/robot_models.yaml`.

![img.png](../_static/image/robot_model_yml.png)

## Usage

### Add robot to config

Add a robot to config:

```yaml
simulator:
  physics_dt: 1/240
  rendering_dt: 1/240

env:
  bg_type: null

render:
  render: true

tasks:
- type: "SingleInferenceTask"
  name: "h1_locomotion"
  env_num: 1
  offset_size: 1.0
  robots:  # Add robots here
  - name: h1
    prim_path: "/World/h1"
    type: "HumanoidRobot"
    position: [.0, .0, 1.05]
    scale: [1, 1, 1]
```

Done.

### Run demo

try this demo:

```python
from grutopia.core.config import SimulatorConfig
from grutopia.core.env import BaseEnv

file_path = f'{path/to/your/config}'
sim_config = SimulatorConfig(file_path)

env = BaseEnv(sim_config, headless=False)
import numpy as np

while env.simulation_app.is_running():

    obs = env.step(actions=env_actions)
env.simulation_app.close()
```

It runs, but the robot doesn't move.

We need to add controller for this robot to make it move.

See [how to use controller](./how-to-use-controller.md)
