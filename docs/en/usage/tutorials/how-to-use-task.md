# How to Use Task

> This tutorial guides you on how to run a task.

## Pre-defined Tasks

The directory [`grutopia_extension/tasks/__init__.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia_extension/tasks/__init__.py) contains a list of all our pre-defined tasks:

```Python
from grutopia_extension.tasks import (
    manipulation_task,
    finite_step_task,
    single_inference_task,
)
```

We can also review the configuration of each task in [`grutopia_extension/configs/tasks/__init__.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia_extension/configs/tasks/__init__.py).


## How to Use Task

To use an existing task within GRUtopia, you can simply use the corresponding type of task config in the runtime configuration as following:

```{code-block} python
:emphasize-lines: 16-22

from grutopia.core.config import Config, SimConfig
from grutopia.core.gym_env import Env
from grutopia.core.runtime import SimulatorRuntime
from grutopia.core.util import has_display
from grutopia.macros import gm
from grutopia_extension import import_extensions
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

env.simulation_app.close()
```

<video width="720" height="405" controls>
    <source src="../../_static/video/tutorial_use_task.webm" type="video/webm">
</video>

In the above example, we use `SingleInferenceTaskCfg` to specify the type of task. For each type of task the corresponding episode config type (`SingleInferenceEpisodeCfg` in the example) should be used to specify episode-wise configurations, such as the scene assets to use in each episode.
