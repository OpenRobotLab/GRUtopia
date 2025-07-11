import numpy as np

from grutopia.core.object import BaseObject
from grutopia.core.scene.scene import IScene
from grutopia_extension.configs.objects import VisualCubeCfg


@BaseObject.register('VisualCube')
class VisualCube(BaseObject):
    def __init__(self, config: VisualCubeCfg, scene: IScene):
        super().__init__(config, scene)
        self._config = config

    def set_up_to_scene(self, scene: IScene):
        from omni.isaac.core.objects.cuboid import VisualCuboid

        scene.add(
            VisualCuboid(
                prim_path=self._config.prim_path,
                name=self._config.name,
                position=np.array(self._config.position),
                scale=np.array([self._config.scale]),
                color=np.array([self._config.color]),
            )
        )  # noqa: F401,F403
