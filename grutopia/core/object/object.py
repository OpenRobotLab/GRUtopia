from grutopia.core.config import ObjectCfg as ObjectConfig
from grutopia.core.config import Simulator


class IObject:
    """
    Object common class.

    Args:
        config (ObjectConfig): The configuration of the object.
    """

    objs = {}

    def __init__(self, config: ObjectConfig):
        self._config = config

    @classmethod
    def register(cls, object_type: str, simulator_type: str):
        """
        Register an object class

        Args:
            object_type (str): The type of the object.
            simulator_type (str): The type of the simulator.

        Returns:
            A function to register the object class.
        """

        def wrapper(subclass):
            """
            Register the object class.
            """
            cls.objs[(object_type, simulator_type)] = subclass
            return subclass

        return wrapper

    @classmethod
    def create(cls, config: ObjectConfig, simulator_type: str = Simulator.ISAACSIM.value) -> 'IObject':
        """
        Create an object based on the given configuration and simulator type.

        Args:
            config (ObjectConfig): configuration of the objects
            simulator_type (str): The type of the simulator.

        Returns:
            IObject: The created object.
        """

        if simulator_type == Simulator.ISAACSIM.value:

            import grutopia.core.object.isaacsim  # noqa

            subclass = IObject.objs.get((config.type, simulator_type))
            if subclass:
                return subclass(config)
            else:
                raise ValueError(f'Invalid object type: {config.type}')
        else:
            raise ValueError(f'Invalid simulator_type: {simulator_type}')
