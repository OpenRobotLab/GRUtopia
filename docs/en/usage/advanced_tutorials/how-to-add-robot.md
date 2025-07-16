# How to Add Custom Robot

> This tutorial guides you on how to add a custom robot.

To add a custom robot, you need to:
- Create a config class for robot config, inheriting from the `grutopia.core.config.robot.RobotCfg`.
- Create a class for robot, inheriting from the `grutopia.core.robot.BaseRobot`.

## Prepare USD of robot

We use [USD](https://openusd.org/release/index.html) file to represent a robot. If you have the URDF/MJCF file of the robot, you can refer to the links below to convert it to a USD file:

- Import URDF: https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_import_urdf.html
- Import MJCF: https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_import_mjcf.html

## Create Config Class

Here's an example of a config class for a robot:

```Python
from grutopia.core.config import RobotCfg

class DemoRobotCfg(RobotCfg):
    # meta info
    name: str = 'demo'
    type: str = 'DemoRobot'
    prim_path: str = '/demo'
    usd_path: Optional[str] = '/data/assets/demo_robot.usd'
```

- name: name of the robot, must be unique in one episode
- type: type of the robot, must be unique and same with the type of robot class
- prim_path: prim path of the robot, relative to the env root path
- usd_path: USD file path, absolute path is preferred to run your code from any working directory

More fields can be added if more attributes are configurable.

Generally, when creating a new config class for robot, reasonable default values for required fields should be specified to avoid validating error, and robot specific config fields can be added when necessary.

## Create Robot Class

In the simplest scenario, the following methods are required to be implemented in your robot class:

```python
from grutopia.core.config import RobotCfg

class DemoRobotCfg(RobotCfg):
    # meta info
    name: str = 'demo'
    type: str = 'DemoRobot'
    prim_path: str = '/demo'
    usd_path: Optional[str] = '/data/assets/demo_robot.usd'

from omni.isaac.core.scenes import Scene
from grutopia.core.robot.robot import BaseRobot

@BaseRobot.register('DemoRobot')  # Register this robot to grutopia
class DemoRobot(BaseRobot):
    def __init__(self, config: DemoRobotCfg, scene: Scene):
        """Initialize the robot with the given config.

        Args:
            config (DemoRobotCfg): config for the robot, should be a instance of corresponding config class.
            scene (Scene): current scene.
        """

    def apply_action(self, action: dict):
        """Apply actions of controllers to robot.

        Args:
            action (dict): action dict.
              key: controller name.
              value: corresponding action array.
        """

    def get_obs(self) -> dict:
        """Get observation of robot, including controllers, sensors, and world pose.
        """
```

The `apply_action` method are used to apply the provided actions, and `get_obs` to obtain the robot's current observations in each step.

For complete list of robot methods, please refer to the [Robot API documentation](../../api/robot.rst).

Please note that the registration of the robot class is done through the `@BaseRobot.register` decorator, and the registered name should match the value of `type` field within the corresponding robot config class (here is `DemoRobot`).

Since a robot typically consists of joints and rigid bodies, [Robot](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#robots) class from Isaac Sim, which derives from [Articulation](https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#articulations) class, are likely to be used in robot implementation. The Articulation class provides high-level encapsulation for [articulated object](https://docs.omniverse.nvidia.com/extensions/latest/ext_physics/articulations.html).

An example of robot class implementation is shown as following:

```python
from omni.isaac.core.robots.robot import Robot as IsaacRobot
from omni.isaac.core.scenes import Scene
from omni.isaac.core.utils.stage import add_reference_to_stage

from grutopia.core.config.robot import RobotUserConfig as Config
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.util import log

@BaseRobot.register('DemoRobot')  # Register this robot to grutopia
class DemoRobot(BaseRobot):

    def __init__(self, config: DemoRobotCfg, scene: Scene):  # Use the config class for this robot
        super().__init__(config, scene)
        self._sensor_config = config.sensors
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        usd_path = config.usd_path

        add_reference_to_stage(prim_path=prim_path, usd_path=os.path.abspath(usd_path))
        self.isaac_robot = IsaacRobot(
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
        # More initialization here...

    def apply_action(self, action: dict):
        """
        Args:
            action (dict): inputs for controllers.
        """
        for controller_name, controller_action in action.items():
            if controller_name not in self.controllers:
                log.warning(f'unknown controller {controller_name} in action')
                continue
            controller = self.controllers[controller_name]
            control = controller.action_to_control(controller_action)
            self.isaac_robot.apply_action(control)

    def get_obs(self):
        """
        Set the observation you need here.
        """
        position, orientation = self._robot_base.get_world_pose()

        # custom
        obs = {
            'position': position,
            'orientation': orientation,
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

You can check the implementations of our robots under [`grutopia_extension/robots/`](https://github.com/OpenRobotLab/GRUtopia/tree/main/grutopia_extension/robots).
