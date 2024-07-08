from typing import Optional, Sequence

import numpy as np
from omni.isaac.core.prims import GeometryPrim, RigidPrim
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.physx.scripts import utils

from grutopia.core.scene.object import ObjectCommon, ObjectConfig, Scene


@ObjectCommon.register('UsdObject')
class UsdObject(ObjectCommon):

    def __init__(self, config: ObjectConfig):
        super().__init__(config=config)
        self._config = config

    def set_up_scene(self, scene: Scene):
        if self._config.collider:
            scene.add(
                RigidObject(usd_path=self._config.usd_obj_param.usd_path,
                            prim_path=self._config.prim_path,
                            name=self._config.name,
                            position=np.array(self._config.position),
                            scale=np.array([self._config.scale])))  # noqa: F401,F403
        else:
            scene.add(
                GeometryObject(usd_path=self._config.usd_obj_param.usd_path,
                               prim_path=self._config.prim_path,
                               name=self._config.name,
                               position=np.array(self._config.position),
                               scale=np.array([self._config.scale])))  # noqa: F401,F403


class RigidObject(RigidPrim):

    def __init__(self,
                 prim_path: str,
                 usd_path: str,
                 name: str = 'custom_obj',
                 position: Optional[np.ndarray] = None,
                 translation: Optional[np.ndarray] = None,
                 orientation: Optional[np.ndarray] = None,
                 scale: Optional[np.ndarray] = None,
                 visible: Optional[bool] = None,
                 mass: Optional[float] = None,
                 density: Optional[float] = None,
                 linear_velocity: Optional[Sequence[float]] = None,
                 angular_velocity: Optional[Sequence[float]] = None,
                 collider: Optional[bool] = True) -> None:
        if not is_prim_path_valid(prim_path):
            if mass is None:
                mass = 1
        prim = add_reference_to_stage(usd_path, prim_path)
        if collider:
            utils.setCollider(prim, approximationShape=None)
        RigidPrim.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            mass=mass,
            density=density,
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
        )


class GeometryObject(GeometryPrim):

    def __init__(
        self,
        prim_path: str,
        usd_path: str,
        name: str = 'visual_cube',
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        color: Optional[np.ndarray] = None,
        size: Optional[float] = None,
    ) -> None:
        add_reference_to_stage(usd_path, prim_path)
        self.size = size
        self.color = color
        GeometryPrim.__init__(
            self,
            prim_path=prim_path,
            name=name,
            position=position,
            translation=translation,
            orientation=orientation,
            scale=scale,
            visible=visible,
            collision=False,
        )
