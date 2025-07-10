from abc import ABC, abstractmethod

from grutopia.core.config import Simulator
from grutopia.core.runtime.task_runtime import TaskRuntime


class IScene(ABC):
    """Abstract base class for all Scene classes."""

    def __init__(self):
        return

    @abstractmethod
    def load(self, runtime: TaskRuntime):
        """Load the scene."""
        raise NotImplementedError()

    @abstractmethod
    def add(self, target: any):
        """Add an object to the scene."""
        raise NotImplementedError()

    @abstractmethod
    def remove(self, target: any, registry_only: bool = False):
        """Remove an object from the scene."""
        raise NotImplementedError()

    @abstractmethod
    def object_exists(self, target: any) -> bool:
        """Check if an object exists in the scene registry."""
        raise NotImplementedError()

    @abstractmethod
    def unwrap(self):
        """Unwraps the scene to its base form."""
        raise NotImplementedError()

    @classmethod
    def create(cls, simulator_type: str = Simulator.ISAACSIM.value) -> 'IScene':
        """Factory method to create IScene instances based on simulator_type."""
        if simulator_type == Simulator.ISAACSIM.value:
            from grutopia.core.scene.isaacsim.scene import IsaacsimScene

            return IsaacsimScene()
        else:
            raise ValueError(f'Invalid simulator_type: {simulator_type}')
