from typing import Optional, Tuple

import numpy as np
from omni.isaac.sensor import Camera

from grutopia.core.wrapper.pose_mixin import PoseMixin


class IsaacCamera(PoseMixin):
    def __init__(
        self,
        prim_path: str,
        name: str = 'camera',
        frequency: Optional[int] = None,
        dt: Optional[str] = None,
        resolution: Optional[Tuple[int, int]] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        render_product_path: str = None,
    ):
        self._camera = Camera(
            prim_path,
            name,
            frequency,
            dt,
            resolution,
            position,
            orientation,
            translation,
            render_product_path,
        )
        super().__init__()

    def unwrap(self):
        return self._camera

    def get_rgba(self):
        """
        Retrieves the RGBA values from the camera.

        Returns:
            numpy.ndarray: The RGBA image captured by the camera.
        """
        return self._camera.get_rgba()

    def get_rgb(self):
        """
        Retrieves the RGB image from the camera.

        Returns:
            numpy.ndarray: The RGB image captured by the camera.
        """
        return self._camera.get_rgb()
