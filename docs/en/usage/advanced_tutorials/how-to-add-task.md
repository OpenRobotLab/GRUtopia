# How to Add Custom Task

> This tutorial guides you on how to add a custom task

## 1. Defining a New Task
Before adding a new task, the following points should be figured out:

- Task Objective: What does the task meant to do?
- Task Termination: Will the task end, and how do we determine that?
- Metrics Calculation: What metrics need to be calculated for the task?


Now we want to add a new task named `FiniteStepTask`. Here's the answers for `FiniteStepTask` based on the above points:

- Objective: Do something until a certain number of steps reached.
- Termination:
  - The Task will end.
  - End Criteria: The task will end after finite steps.
- Metrics Calculation:
  - Record the total distance a robot moves from start to finish.
  - (Additional metrics as needed.)

To add a custom task, you need to:

- Create a config class for episode, inheriting from the `grutopia.core.config.task.EpisodeCfg`.
- Create a config class for task, inheriting from the `grutopia.core.config.task.TaskCfg`.
- Create a class for task, inheriting from the `grutopia.core.task.BaseTask`.

For the differences between the two config classes, follow the rules below:

- The configs that don't vary across all the episodes should be defined in **task** config class.
- The configs that may vary between episodes should be defined in **episode** config class.
- The task config class should always include a list of corresponding episode configs.

## 2. Create Config Classes

Here's an example of the task and episode config classes for the `FiniteStepTask`:

```python
from typing import List, Optional

from grutopia.core.config.task import TaskCfg


class FiniteStepTaskCfg(TaskCfg):
  type: Optional[str] = 'FiniteStepTask'
  max_step: Optional[int] = 500

```

## 3. Create Task Class

What is a runtime?

```Python
from omni.isaac.core.scenes import Scene

from grutopia.core.task_config_manager.base import TaskCfg
from grutopia.core.task import BaseTask


@BaseTask.register('FiniteStepTask')
class FiniteStepTask(BaseTask):
  def __init__(self, task_config: TaskCfg, scene: Scene):
    super().__init__(task_config, scene)
    self.stop_count = 0
    self.max_step = 500  # default max_step
    # ======= Validate task setting =======
    if not task_config.task_settings:
      pass
    elif not isinstance(task_config.task_settings, dict):
      raise ValueError('task_settings of FiniteStepTask must be a dict')
    if 'max_step' in task_config.task_settings:
      self.max_step = task_config.task_settings['max_step']
    # ======= Validate task setting =======

  def is_done(self) -> bool:
    self.stop_count += 1
    return self.stop_count > self.max_step

```
- The `is_done` method is overridden based on the termination condition defined [above](#1-defining-a-new-task).
- This task is ready to use, but it will not output any results by itself. To make it work in practice, you must define Metrics and configure them.


## 4. Create Metrics Config Class


To add a custom metric, you need to:
- Create a config class for metric config, inheriting from the `grutopia.core.config.metric.MetricUserConfig`.
- Create a class for metric, inheriting from the `grutopia.core.task.metric.BaseMetric`.

Here's an example of a config class for a metric:

```python
# This is also the simplest configuration.
from typing import Optional

from grutopia.core.config.metric import MetricUserConfig


class SimpleMetricCfg(MetricUserConfig):
    type: Optional[str] = 'SimpleMetric'
```


## 5. Create Metrics Class

In this doc, we demonstrate a simple metrics used to track the total distance a robot moves.

```Python
import numpy as np
from pydantic import BaseModel

from grutopia.core.config.metric import MetricCfg
from grutopia.core.task_config_manager.base import TaskRuntime
from grutopia.core.task.metric import BaseMetric
from grutopia.core.util import log


class SimpleMetricParam(BaseModel):
  robot_name: str


@BaseMetric.register('SimpleMetric')
class SimpleMetric(BaseMetric):
  """
  Calculate the total distance a robot moves
  """

  def __init__(self, config: MetricCfg, task_runtime: TaskRuntime):
    super().__init__(config, task_runtime)
    self.distance: float = 0.0
    self.position = None
    self.param = SimpleMetricParam(**config.metric_config)
    _robot_name = self.param.robot_name
    self.robot_name = (
            _robot_name + '_' + str(self.task_runtime.env.env_id)
    )  # real robot name in isaac sim: {robot_name}_{env_id}

  def reset(self):
    self.distance = 0.0

  def update(self, task_obs: dict):
    """
    This function is called at each world step.
    """
    if self.position is None:
      self.position = task_obs[self.robot_name]['position']
      return
    self.distance += np.linalg.norm(self.position - task_obs[self.robot_name]['position'])
    self.position = task_obs[self.robot_name]['position']
    return

  def calc(self):
    """
    This function is called to calculate the metrics when the episode is terminated.
    """
    log.info('SimpleMetric calc() called.')
    return self.distance

```
- The `update` method will be invoked after every step.
- The `calc` method will be invoked at the end of an episode.
- The `reset` method be invoked when an episode is reset. Reset within one episode is not supported yet, so this method won't be used at the time.


## 6. Task Usage Preview
To use the custom task and metrics, you can simply include them in the configuration settings as follows

```Python
...
config = Config(
    simulator=SimConfig(physics_dt=1 / 240, rendering_dt=1 / 240, use_fabric=False),
    task_config=FiniteStepTaskCfg(
        task_settings={'max_step': 300},
        metrics=[
            SimpleMetricCfg(metric_config={'robot_name': 'h1'}),
            # Add more metrics here.
        ],
        metrics_save_path='./h1_simple_metric.jsonl',
        episodes=[
            ...
        ],
    ),
)

sim_runtime = SimulatorRuntime(config_class=config)

import_extensions()
# import custom extensions here.

env = Env(sim_runtime)
...
```

You can also run the [`h1_traveled_distance.py`](https://github.com/OpenRobotLab/GRUtopia/blob/main/grutopia/demo/h1_traveled_distance.py) in the demo directly.

And you can check result in `./h1_simple_metric.jsonl` after the task finishes.

```json
{"SimpleMetric": 0.7508679775492055, "normally_end": true}
```
