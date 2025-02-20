# How to Add Custom Task

> This tutorial will show you how to add a custom task

## 1. Defining a New Task
Before adding a new task, we need to clarify the following points:

- Task Naming: What will the task be called?
- Task Objective: What specific Objective will the task achieve?
- Task Termination: Will the task end, and how will we determine that?
- Metrics Calculation: What metrics need to be calculated for the task?


Here's how we define our SocialNavigationTask based on the above points:
- Name: SocialNavigationTask
- Objective: Benchmark
- Termination:
  - The Task will end.
  - End Criteria: The task will conclude either after 600 steps or when a termination signal is received from Datahub.
- Metrics Calculation:
  - Record the time taken for each step.
  - Record the total distance a robot moves from start to finish (including the path detail).
  - Additional metrics as needed.

## 2. Create Custom Task Inheriting from  `grutopia.core.task.BaseTask`

```Python
import traceback
from typing import Any, Dict

from omni.isaac.core.scenes import Scene

from grutopia.core.datahub import DataHub
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task import BaseTask
from grutopia.core.util import log
from grutopia_extension.configs.tasks.social_navigation_task import (
    SocialNavigationExtra,
    TaskSettingCfg,
)


@BaseTask.register('SocialNavigationTask')
class SocialNavigationTask(BaseTask):
    def __init__(self, runtime: TaskRuntime, scene: Scene):
        super().__init__(runtime, scene)
        self.step_counter = 0
        if isinstance(runtime.task_settings, TaskSettingCfg):
            self.settings = runtime.task_settings
        else:
            raise ValueError('task_settings must be a TaskSettingCfg')
        if isinstance(runtime.extra, SocialNavigationExtra):
            self.episode_meta = runtime.extra
        else:
            raise ValueError('extra must be a SocialNavigationExtra')
        log.info(f'task_settings: max_step       : {self.settings.max_step}.)')
        # Episode
        log.info(f'Episode meta : question         : {self.episode_meta.question}.)')


    def is_done(self) -> bool:
        self.step_counter = self.step_counter + 1
        return DataHub.get_episode_finished(self.runtime.name) or self.step_counter > self.settings.max_step

    def individual_reset(self):
        for name, metric in self.metrics.items():
            metric.reset()

```
- The `is_done` method has been overridden based on the End Criteria defined above.
- The purpose of `individual_reset` method is to define actions that need to be taken when resetting the task without loading a new episode. In this case, it is optional.
- This task is ready for use, but it will not output any results by itself. To make it work in practice, you must define Metrics and configure them in the configuration file.
- The `TaskSettingCfg` defined in `grutopia_extension.configs.tasks.social_navigation_task` indicates the configuration required for each task. Each individual task may have a distinct configuration. For benchmark tasks, the primary configuration necessary is the timeout duration, which can be set as follows:
```
class TaskSettingCfg(BaseModel):
    max_step: int

class SocialNavigationTaskCfg(TaskCfg):
    ...
    task_settings: TaskSettingCfg
    ...
```

## 3. Create Custom Metrics Inheriting from  `grutopia.core.task.metric.BaseMetric`

In this doc, we demonstrate a simple metrics used to track the total distance a robot moves.

```Python
import numpy as np
from pydantic import BaseModel

from grutopia.core.config.metric import MetricCfg
from grutopia.core.runtime.task_runtime import TaskRuntime
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
        # log.info(f'distance: {self.distance}')
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
- The `reset` method be invoked when an episode is reset. Currently episodes will be switched when they end rather than reset, so this method won't be used at this time.

We also need to define the metrics config in the  `grutopia_extension/configs/metrics` directory.
```python
from typing import Optional

from grutopia.core.config.metric import MetricCfg

class SimpleMetricCfg(MetricCfg):
    type: Optional[str] = 'SimpleMetric'
```

## 3. Task Usage Preview
To use the custom task and metrics, you can simply include them in the configuration settings as follows

```Python
...
config = Config(
   ...
    task_config=SocialNavigationTaskCfg(
        metrics=[
            SimpleMetricCfg(),
        ],
        task_settings=SocialNavigationTaskSetting(max_step=6000),
...
)
```
