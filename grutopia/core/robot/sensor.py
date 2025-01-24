from abc import ABC, abstractmethod
from functools import wraps
from typing import Dict, List

from grutopia.core.config.robot import RobotCfg, SensorModel
from grutopia.core.robot.robot import BaseRobot, Scene
from grutopia.core.util import log


class BaseSensor(ABC):
    """Base class of sensor."""

    sensors = {}

    def __init__(self, config: SensorModel, robot: BaseRobot, scene: Scene):
        """Initialize the sensor.

        Args:
            config (SensorModel): merged config (from user config and robot model) of the sensor.
            robot (BaseRobot): robot owning the sensor.
            scene (Scene): scene from isaac sim.
        """
        if config.name is None:
            raise ValueError('must specify sensor name.')
        self.name = config.name
        self.config = config
        self._scene = scene
        self._robot = robot

    @abstractmethod
    def init(self):
        raise NotImplementedError()

    @abstractmethod
    def get_data(self) -> Dict:
        """Get data from sensor.

        Returns:
            Dict: data dict of sensor.
        """
        raise NotImplementedError()

    @abstractmethod
    def reset(self):
        raise NotImplementedError()

    def cleanup(self):
        """
        Operations that need to be cleaned up before switching scenes (or resetting)
        """
        pass

    @classmethod
    def register(cls, name: str):
        """
        Register a sensor class with the given name(decorator).
        Args:
            name(str): name of the sensor class.
        """

        def decorator(sensor_class):
            cls.sensors[name] = sensor_class

            @wraps(sensor_class)
            def wrapped_function(*args, **kwargs):
                return sensor_class(*args, **kwargs)

            return wrapped_function

        return decorator


def create_sensors(robot_model: RobotCfg, robot: BaseRobot, scene: Scene) -> Dict[str, BaseSensor]:
    """Create all sensors of one robot.

    Args:
        robot_model (RobotModel): model of the robot.
        robot (BaseRobot): robot instance.
        scene (Scene): scene from isaac sim.

    Returns:
        Dict[str, BaseSensor]: dict of sensors with sensor name as key.
    """
    sensor_map = {}
    if robot_model.sensors is not None:
        sensors: List[SensorModel] = robot_model.sensors
        for sensor in sensors:
            sensor_cls = BaseSensor.sensors[sensor.type]
            sensor_ins = sensor_cls(sensor, robot=robot, name=sensor.name, scene=scene)
            sensor_map[sensor.name] = sensor_ins
            log.debug(f'==================== {sensor.name} loaded==========================')

    return sensor_map
