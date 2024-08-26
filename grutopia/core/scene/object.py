from functools import wraps

from omni.isaac.core.scenes import Scene

from grutopia.core.config import Object as ObjectConfig


class ObjectCommon:
    """
    Object common class.
    """
    objs = {}

    def __init__(self, config: ObjectConfig):
        self._config = config

    def set_up_scene(self, scene: Scene):
        raise NotImplementedError

    @classmethod
    def register(cls, name: str):
        """
        Register an object class with the given name(decorator).

        Args:
            name(str): name of the object
        """

        def decorator(object_class):
            cls.objs[name] = object_class

            @wraps(object_class)
            def wrapped_function(*args, **kwargs):
                return object_class(*args, **kwargs)

            return wrapped_function

        return decorator


def create_object(config: ObjectConfig):
    """
    Create an object.
    Args:
        config (ObjectConfig): configuration of the objects
    """
    assert config.type in ObjectCommon.objs, 'unknown objects type {}'.format(config.type)
    obj: ObjectCommon = ObjectCommon.objs[config.type](config)
    return obj
