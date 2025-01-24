import numpy as np
from omni.isaac.core.objects.cuboid import VisualCuboid

from grutopia.core.scene.object import ObjectCommon, Scene
from grutopia_extension.configs.objects import VisualCubeCfg


@ObjectCommon.register('VisualCube')
class VisualCube(ObjectCommon):
    def __init__(self, config: VisualCubeCfg):
        super().__init__(config=config)
        self._config = config

    def set_up_scene(self, scene: Scene):
        scene.add(
            VisualCuboid(
                prim_path=self._config.prim_path,
                name=self._config.name,
                position=np.array(self._config.position),
                scale=np.array([self._config.scale]),
                color=np.array([self._config.color]),
            )
        )  # noqa: F401,F403
