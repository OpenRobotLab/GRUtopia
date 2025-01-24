import numpy as np
from omni.isaac.core.objects import DynamicCuboid

from grutopia.core.scene.object import ObjectCommon, Scene
from grutopia_extension.configs.objects import DynamicCubeCfg


@ObjectCommon.register('DynamicCube')
class DynamicCube(ObjectCommon):
    def __init__(self, config: DynamicCubeCfg):
        super().__init__(config=config)
        self._config = config

    def set_up_scene(self, scene: Scene):
        scene.add(
            DynamicCuboid(
                prim_path=self._config.prim_path,
                name=self._config.name,
                position=np.array(self._config.position),
                orientation=np.array(self._config.orientation),
                scale=np.array(self._config.scale),
                color=np.array(self._config.color),
            )
        )
