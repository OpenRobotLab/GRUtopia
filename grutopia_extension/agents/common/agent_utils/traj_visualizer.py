# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

from typing import Any, List, Union

import cv2
import numpy as np


class TrajectoryVisualizer:
    _num_drawn_points: int = 1
    _cached_path_mask: Union[np.ndarray, None] = None
    _origin_in_img: Union[np.ndarray, None] = None
    _pixels_per_meter: Union[float, None] = None
    agent_line_length: int = 10
    agent_line_thickness: int = 3
    path_color: tuple = (0, 255, 0)
    path_thickness: int = 3
    scale_factor: float = 1.0

    def __init__(self, origin_in_img: np.ndarray, pixels_per_meter: float):
        self._origin_in_img = origin_in_img
        self._pixels_per_meter = pixels_per_meter

    def reset(self) -> None:
        self._num_drawn_points = 1
        self._cached_path_mask = None

    def draw_trajectory(
        self,
        img: np.ndarray,
        camera_positions: Union[np.ndarray, List[np.ndarray]],
        camera_yaw: float,
    ) -> np.ndarray:
        """Draws the trajectory on the image and returns it"""
        img = self._draw_path(img, camera_positions)
        img = self._draw_agent(img, camera_positions[-1], camera_yaw)
        return img

    def _draw_path(self, img: np.ndarray, camera_positions: Union[np.ndarray, List[np.ndarray]]) -> np.ndarray:
        """Draws the path on the image and returns it"""
        if len(camera_positions) < 2:
            return img
        if self._cached_path_mask is not None:
            path_mask = self._cached_path_mask.copy()
        else:
            path_mask = np.zeros(img.shape[:2], dtype=np.uint8)

        for i in range(self._num_drawn_points - 1, len(camera_positions) - 1):
            path_mask = self._draw_line(path_mask, camera_positions[i], camera_positions[i + 1])

        img[path_mask == 255] = self.path_color

        self._cached_path_mask = path_mask
        self._num_drawn_points = len(camera_positions)

        return img

    def _draw_line(self, img: np.ndarray, pt_a: np.ndarray, pt_b: np.ndarray) -> np.ndarray:
        """Draws a line between two points and returns it"""
        # Convert metric coordinates to pixel coordinates
        px_a = self._metric_to_pixel(pt_a)
        px_b = self._metric_to_pixel(pt_b)

        if np.array_equal(px_a, px_b):
            return img

        cv2.line(
            img,
            tuple(px_a[::-1]),
            tuple(px_b[::-1]),
            255,
            int(self.path_thickness * self.scale_factor),
        )

        return img

    def _draw_agent(self, img: np.ndarray, camera_position: np.ndarray, camera_yaw: float) -> np.ndarray:
        """Draws the agent on the image and returns it"""
        px_position = self._metric_to_pixel(camera_position)
        cv2.circle(
            img,
            tuple(px_position[::-1]),
            int(8 * self.scale_factor),
            (255, 192, 15),
            -1,
        )
        heading_end_pt = (
            int(px_position[0] - self.agent_line_length * self.scale_factor * np.cos(camera_yaw)),
            int(px_position[1] - self.agent_line_length * self.scale_factor * np.sin(camera_yaw)),
        )
        cv2.line(
            img,
            tuple(px_position[::-1]),
            tuple(heading_end_pt[::-1]),
            (0, 0, 0),
            int(self.agent_line_thickness * self.scale_factor),
        )

        return img

    def draw_circle(self, img: np.ndarray, position: np.ndarray, **kwargs: Any) -> np.ndarray:
        """Draws the point as a circle on the image and returns it"""
        px_position = self._metric_to_pixel(position)
        cv2.circle(img, tuple(px_position[::-1]), **kwargs)

        return img

    def _metric_to_pixel(self, pt: np.ndarray) -> np.ndarray:
        """Converts a metric coordinate to a pixel coordinate"""
        # Need to flip y-axis because pixel coordinates start from top left
        px = pt * self._pixels_per_meter * np.array([-1, -1]) + self._origin_in_img
        # px = pt * self._pixels_per_meter + self._origin_in_img
        px = px.astype(np.int32)
        return px
