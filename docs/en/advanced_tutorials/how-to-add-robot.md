# How to add custom robot

> This tutorial will show you how to add a robot

## 1. Add isaac sim robot

> Assuming you already have an usd file of a robot, and it has drivable joints.

Create a file in `grutopia_extension/robots`, named `demo_robot.py`. Inherit the robot class from isaac.

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

## 2. Wrap with `grutopia.core.robot.robot`

```Python
from omni.isaac.core.scenes import Scene

from grutopia.core.config.robot import RobotUserConfig as Config
from grutopia.core.robot.robot import BaseRobot
from grutopia.core.config.robot import RobotModel
from grutopia.core.util import log


# Register this robot to grutopia.core
@BaseRobot.register('DemoRobot')
class DemoRobotWrapper(BaseRobot):

    def __init__(self, config: Config, robot_model: RobotModel, scene: Scene):
        super().__init__(config, robot_model, scene)
        self._sensor_config = robot_model.sensors
        self._gains = robot_model.gains
        self._start_position = np.array(config.position) if config.position is not None else None
        self._start_orientation = np.array(config.orientation) if config.orientation is not None else None

        log.debug(f'demo_robot {config.name}: position    : ' + str(self._start_position))
        log.debug(f'demo_robot {config.name}: orientation : ' + str(self._start_orientation))

        usd_path = robot_model.usd_path
        if usd_path.startswith('/Isaac'):
            usd_path = get_assets_root_path() + usd_path

        log.debug(f'demo_robot {config.name}: usd_path         : ' + str(usd_path))
        log.debug(f'demo_robot {config.name}: config.prim_path : ' + str(config.prim_path))

        # Wrap the robot class here.
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

* And there are many other functions in `grutopia.core.robot.robot`, FYI.

## 3. Register at `robot_models`

Add you robot model at `grutopia_extension/robots/robot_models.yaml`

```yaml
- type: "DemoRobotWrapper"
  usd_path: "..."
  controllers:
  - name: "..."
    type: "..."
```

## 4. Add controllers and sensors

See  [how to add controller](./how-to-add-controller.md) and [how to add sensor](./how-to-add-sensor.md)

## 5. Write a demo

See [how to add controller](./how-to-add-controller.md) and [how to add sensor](./how-to-add-sensor.md)
