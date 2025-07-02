from typing import Optional, Sequence

import numpy as np
from omni.isaac.core.prims import RigidPrim

from grutopia.core.util.physics_status_util import (
    get_rigidbody_status,
    set_rigidbody_status,
)
from grutopia.core.wrapper.pose_mixin import PoseMixin


class IsaacRigidBodyPrim(PoseMixin):
    def __init__(
        self,
        prim_path: str,
        name: str = 'rigid_prim',
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        mass: Optional[float] = None,
        density: Optional[float] = None,
        linear_velocity: Optional[np.ndarray] = None,
        angular_velocity: Optional[np.ndarray] = None,
    ):
        self._param = {
            'prim_path': prim_path,
            'name': name,
            'position': position,
            'translation': translation,
            'orientation': orientation,
            'scale': scale,
            'visible': visible,
            'mass': mass,
            'density': density,
            'linear_velocity': linear_velocity,
            'angular_velocity': angular_velocity,
        }
        self._rigid_prim = RigidPrim(**self._param)
        super().__init__()
        self.status = {}

    def unwrap(self):
        return self._rigid_prim

    def save_status(self):
        self.status = get_rigidbody_status(self._rigid_prim)

    def restore_status(self):
        self._rigid_prim = RigidPrim(**self._param)
        if not self.status:
            self.save_status()
            return
        set_rigidbody_status(self._rigid_prim, self.status)

    @property
    def name(self):
        return self._rigid_prim.name

    @property
    def prim_ins(self):
        return self._rigid_prim

    def get_linear_velocity(self):
        return self._rigid_prim.get_linear_velocity()

    def get_angular_velocity(self):
        return self._rigid_prim.get_angular_velocity()

    def initialize(self, physics_sim_view=None) -> None:
        self._rigid_prim.initialize(physics_sim_view)
