# How to Add Custom Robot

> This tutorial will show you how to add a robot. If you wish to use this robot in practice, you need to:
> - Learn [how to add a custom controller](./how-to-add-controller.md) to control the robot you’ve built.
> - Learn [how to add a custom sensor](./how-to-add-sensor.md) to collect data.

## 1. Inherit from `omni.isaac.core.robots.robot`

Create a file in `grutopia_extension/robots`, named `demo_robot.py`, in which inherits the robot class `omni.isaac.core.robots.robot`.

```Python
from omni.isaac.core.robots.robot import Robot as IsaacRobot
from omni.isaac.core.utils.stage import add_reference_to_stage


class DemoRobot(IsaacRobot):

    def __init__(self,
                 prim_path: str,
                 usd_path: str,
                 name: str,
                 position: np.ndarray = None,
                 orientation: np.ndarray = None,
                 scale: np.ndarray = None):
        add_reference_to_stage(prim_path=prim_path, usd_path=os.path.abspath(usd_path))
        super().__init__(prim_path=prim_path, name=name, position=position, orientation=orientation, scale=scale)
        # Set robot-specific parameters/attributes here
```

The above class has the following functions:

- It will load the robot files from the usd_path into the prim_path location in the stage using the `add_reference_to_stage` method.
- It will initialize the robot's parameters using the `IsaacRobot` class, including prim_path, name, configuration settings, scale, and more.
- The commented section can be replaced with any additional parameters required to define the robot's characteristics.

## 2. Wrap `Isaac-based robot` with `grutopia.core.robot.BaseRobot`

After creating an Isaac-based robot, we need to wrap this robot using the GRUtopia robot class `BaseRobot`. This allows the robot to connect to our platform, which includes automatic registration, flow control, and other essential functionalities.

```Python
from omni.isaac.core.scenes import Scene

from grutopia.core.config.robot import RobotUserConfig as Config
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.config.robot import RobotModel
from grutopia.core.util import log


# Register this robot to grutopia.core
@BaseRobot.register('DemoRobotWrapper')
class DemoRobotWrapper(BaseRobot):

    def __init__(self, config: Config, robot_model: RobotModel, scene: Scene):
        super().__init__(config, robot_model, scene)
        self._sensor_config = robot_model.sensors
        self._gains = robot_model.gains
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        usd_path = robot_model.usd_path
        if usd_path.startswith('/Isaac'):
            usd_path = get_assets_root_path() + usd_path

        self.isaac_robot = DemoRobot(
            prim_path=config.prim_path,
            name=config.name,
            position=self._start_position,
            orientation=self._start_orientation,
            usd_path=usd_path,
        )

        self._robot_scale = np.array([1.0, 1.0, 1.0])
        if config.scale is not None:
            self._robot_scale = np.array(config.scale)
            self.isaac_robot.set_local_scale(self._robot_scale)

        # Add the attr you want here.

    ...

    def apply_action(self, action: dict):
        """
        Args:
            action (dict): inputs for controllers.
        """
        for controller_name, controller_action in action.items():
            if controller_name not in self.controllers:
                log.warn(f'unknown controller {controller_name} in action')
                continue
            controller = self.controllers[controller_name]
            control = controller.action_to_control(controller_action)
            self.isaac_robot.apply_actuator_model(control, controller_name, self.joint_subset)

    def get_obs(self):
        """
        Set the observation you need here.
        """

        # custom
        position, orientation = self._robot_base.get_world_pose()
        obs = {
            'position': position,
            'orientation': orientation,
        }

        # common
        for c_obs_name, controller_obs in self.controllers.items():
            obs[c_obs_name] = controller_obs.get_obs()
        for sensor_name, sensor_obs in self.sensors.items():
            obs[sensor_name] = sensor_obs.get_data()
        return obs
```
Please note:
- This class will automatically register with GRUtopia under the name DemoRobotWrapper (see line 10).
- As shown in the code, the primary responsibilities of this class are:
  1. Setting up the robot's parameters
  2. Defining the robot's actions
  3. Specifying the format of the output observations and their data sources.
- Additionally, it can define many other methods to implement advanced functionalities. Further details can be found in `grutopia.core.robot.robot`.

## 3. Create Robot's Configuration

Create a configuration file for your custom robot in the directory `grutopia_extension/config/robots/`. This config file should include supported controllers, sensors, parameters, and more.

The following only provides a brief demonstration, please refer to `grutopia_extension/config/robots/humanoid.py` for a complete robot configuration.

```Python
dummy_move = DummyMoveControllerCfg(
    ...
)

camera = DepthCameraCfg(
    ...
)

class DemoRobotCfg(RobotCfg):
    # meta info
   ...

```
