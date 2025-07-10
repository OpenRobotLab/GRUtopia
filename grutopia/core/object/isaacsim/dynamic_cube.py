import numpy as np
from omni.isaac.core.objects import DynamicCuboid

from grutopia.core.config import Simulator
from grutopia.core.config.object import DynamicCubeCfg
from grutopia.core.object.object import IObject


@IObject.register('DynamicCube', Simulator.ISAACSIM.value)
class IsaacsimDynamicCube(IObject):
    def __init__(self, config: DynamicCubeCfg):
        super().__init__(config=config)
        self._cube = DynamicCuboid(
            prim_path=self._config.prim_path,
            name=self._config.name,
            position=np.array(self._config.position),
            orientation=np.array(self._config.orientation),
            scale=np.array(self._config.scale),
            color=np.array(self._config.color),
        )
