# How to Add Custom Controller

> This tutorial guides you on how to add a custom controller for a robot.

Note that the controller cannot be operated independently. It must be used with robot to enable robot to act in the environment.


To add a custom controller, you need to:
- Create a config class for controller config, inheriting from the `grutopia.core.config.robot.ControllerCfg`.
- Create a class for controller, inheriting from the `grutopia.core.robot.controller.BaseController`.

## Create Config Class

Here's an example of a config class for a controller:

```Python
from grutopia.core.config.robot import ControllerCfg


class DemoControllerCfg(ControllerCfg):

    name: str = 'demo_controller'
    type: str = 'DemoController'
    forward_speed: float = 1.0
```

Generally, when creating a new config class, reasonable default values for required fields should be specified, and controller specific config fields can be added when necessary.

## Create Controller Class

In the simplest scenario, the following methods are required to be implemented in your controller class:

```python
import numpy as np
from typing import List, Union
from omni.isaac.core.utils.types import ArticulationAction

from grutopia.core.robot.controller import BaseController
from grutopia.core.robot.robot import BaseRobot


@BaseController.register('DemoController')
class DemoController(BaseController):

    def __init__(self, config: DemoControllerCfg, robot: BaseRobot, scene: Scene):
        """Initialize the controller with the given configuration and its owner robot.

        Args:
            config (DemoControllerCfg): controller configuration.
            robot (BaseRobot): robot owning the controller.
            scene (Scene): scene from isaac sim.
        """

    def action_to_control(self, action: Union[np.ndarray, List]) -> ArticulationAction:
        """Convert input action (in 1d array format) to joint signals to apply.

        Args:
            action (Union[np.ndarray, List]): input control action.

        Returns:
            ArticulationAction: joint signals to apply
        """
```

The `action_to_control` method translates the input action into joint signals to apply in each step.

For complete list of controller methods, please refer to the [Controller API documentation](../../api/robot.rst#module-grutopia.core.robot.controller).

Please note that the registration of the controller class is done through the `@BaseController.register` decorator, and the registered name should match the value of `type` field within the corresponding controller config class (here is `DemoController`).

Sometimes the calculation logic is defined in a method named `forward` to show the input parameters the controller accepts (which is common in our implementations), making it more human-readable. In this case, the `action_to_control` method itself only expands the parameters, and invokes `forward` method to calculate the joint signals.

An example of controller class implementation is shown as following:

```python
from typing import List

import numpy as np
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.types import ArticulationAction

from grutopia.core.robot.controller import BaseController
from grutopia.core.robot.robot import BaseRobot
from grutopia_extension.configs.controllers import DemoControllerCfg


@BaseController.register('DemoController')
class DemoController(BaseController):
    def __init__(self, config: DemoControllerCfg, robot: BaseRobot, scene: Scene) -> None:
        super().__init__(config=config, robot=robot, scene=scene)

    def forward(
        self,
        forward_speed: float = 0,
        rotation_speed: float = 0,
        scaler: float = 1,
    ) -> ArticulationAction:
        if forward_speed == 0 and rotation_speed == 0:
            return ArticulationAction(joint_velocities=np.array([0, 0]))

        forward_basis = np.array([1.0, 1.0])
        spin_basis = np.array([-1.0, 1.0])

        wheel_vel_for = forward_basis * forward_speed
        wheel_vel_rot = spin_basis * rotation_speed
        wheel_vel = wheel_vel_for + wheel_vel_rot

        return ArticulationAction(joint_velocities=wheel_vel)

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
            scaler=1 / self.robot.get_robot_scale()[0],
        )
```

You can check the implementations of our controllers under [`grutopia_extension/controllers/`](https://github.com/OpenRobotLab/GRUtopia/tree/main/grutopia_extension/controllers).
