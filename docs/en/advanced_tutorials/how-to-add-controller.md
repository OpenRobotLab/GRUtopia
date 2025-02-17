# How to Add Custom Controller

> This tutorial will show you how to add a custom controller for a robot

Note that the controller cannot operate independently. It works with robots to enable robots to move, which serves as a prerequisite for the robot's motion.


## 1. Create Controller's Configuration
Create the custom controller's configuration in `grutopia_extension/config/controllers/`. The configuration will cover all parameters that controller needs.

```Python

class DummyMoveControllerCfg(ControllerModel):
    ...
```

## 2. Inherit from `grutopia.core.robot.controller`
`grutopia.core.robot.controller` serves as the base class for all controllers we use. Its primary functions include:

- Validating the input parameters.
- Implementing robot movement through the `forward` method.
- Providing output observations (obs) required by the controller.(Its worth noting that all observations should ideally come from sensors, this has been included only for debugging convenience.)

```python
from datetime import datetime
from typing import Any, Dict, List, Union

import numpy as np
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.types import ArticulationAction

from grutopia.core.datahub.model_data import LogData, ModelData
from grutopia.core.robot.controller import BaseController
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.config.robot import ControllerModel


@BaseController.register('DummyMoveController')
class DummyMoveController(BaseController):

    # Let's define a controller which moves based on input. It accepts a two-element array, where the first element represents the orientation and the second element represents the velocity.
    def __init__(self,config: DummyMoveControllerModel, robot: BaseRobot, scene: Scene) -> None:
        super().__init__(config=config, robot=robot, scene=scene)
        self._user_config = None
        self.counter = 1


    def action_to_control(self, action: Union[np.ndarray, List]) -> ArticulationAction:
        # Validate and mutate the action
        return self.forward(action[])

    def forward(self, action: str) -> ArticulationAction:
        # This function does not include an implementation, as it is intended solely as an example.

        ...
        return ArticulationAction()

    def get_obs(self) -> Dict[str, Any]:
        # return the observations
        ...
        return {}
```

## 3. Controller Usage Preview
Follow the steps outlined in [how to use robot](../tutorials/how-to-use-robot.md) to add the new controller to a robot. Then you can use it as demonstrated below:
```python
...
while env.simulation_app.is_running():
    ...
    env_action = {'dummy_move': np.array([0.0, 1.0])}
    obs, _, _, _, _ = env.step(action=env_action)
    print(obs)
    ...
env.simulation_app.close()
```
