from abc import ABC, abstractmethod
from functools import wraps
from typing import Dict

from grutopia.core.config.robot import RobotUserConfig
from grutopia.core.config.robot.params import SensorParams
from grutopia.core.robot.robot import BaseRobot, Scene
from grutopia.core.robot.robot_model import RobotModel, SensorModel
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
    def sensor_init(self):
        raise NotImplementedError()

    @abstractmethod
    def get_data(self) -> Dict:
        """Get data from sensor.

        Returns:
            Dict: data dict of sensor.
        """
        raise NotImplementedError()

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


def config_inject(params: SensorParams, model: SensorModel) -> SensorModel:
    """Merge sensor config from user config and robot model.

    Args:
        params (SensorParams): user config.
        model (SensorModel): sensor config from robot model.

    Returns:
        SensorModel: merged sensor config.
    """
    if params is None:
        return model
    config = model.dict()
    user = params.dict()
    for k, v in user.items():
        if v is not None:
            config[k] = v
    conf = SensorModel(**config)

    return conf


def create_sensors(config: RobotUserConfig, robot_model: RobotModel, robot: BaseRobot,
                   scene: Scene) -> Dict[str, BaseSensor]:
    """Create all sensors of one robot.

    Args:
        config (RobotUserConfig): user config of the robot.
        robot_model (RobotModel): model of the robot.
        robot (BaseRobot): robot instance.
        scene (Scene): scene from isaac sim.

    Returns:
        Dict[str, BaseSensor]: dict of sensors with sensor name as key.
    """
    sensor_map = {}
    if robot_model.sensors is not None:
        available_sensors = {a.name: a for a in robot_model.sensors}
        for sensor_name, sensor in available_sensors.items():
            if sensor.type not in BaseSensor.sensors:
                raise KeyError(f'unknown sensor type "{sensor.type}"')
            sensor_cls = BaseSensor.sensors[sensor.type]
            # Find if user param exists for this sensor.
            param = None
            if config.sensor_params is not None:
                for p in config.sensor_params:
                    if p.name == sensor_name:
                        param = p
                        break

            sensor_ins = sensor_cls(config=config_inject(param, sensor), robot=robot, name=sensor_name, scene=scene)
            sensor_map[sensor_name] = sensor_ins
            sensor_ins.sensor_init()
            log.debug(f'==================== {sensor_name} loaded==========================')

    return sensor_map
