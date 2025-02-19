# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.
import os

import cv2
import matplotlib.pyplot as plt
import numpy as np
from agent_utils.geometry_utils import (
    extract_camera_pos_zyxrot,
    get_extrinsic_matrix,
    get_world_points_from_image_coords,
)
from agent_utils.img_utils import fill_small_holes
from depth_camera_filtering import filter_depth
from frontier_exploration.frontier_detection import detect_frontier_waypoints
from frontier_exploration.utils.fog_of_war import reveal_fog_of_war
from modules.mapping.base_map import BaseMap
from scipy.spatial.distance import pdist, squareform


class ObstacleMap(BaseMap):
    """Generates two maps; one representing the area that the robot has explored so far,
    and another representing the obstacles that the robot has seen so far.
    """

    _map_dtype: np.dtype = np.dtype(bool)
    _frontiers_px: np.ndarray = np.array([])
    frontiers: np.ndarray = np.array([])
    radius_padding_color: tuple = (100, 100, 100)

    def __init__(
        self,
        min_height: float,
        max_height: float,
        agent_radius: float,
        area_thresh: float = 3.0,  # square meters
        hole_area_thresh: int = 100000,  # square pixels
        size: int = 1000,
        pixels_per_meter: int = 20,
    ):
        super().__init__(size, pixels_per_meter)
        self.explored_area = np.zeros((size, size), dtype=bool)
        self._map = np.zeros((size, size), dtype=bool)
        self.seen_map = np.zeros((size, size), dtype=bool)
        self._navigable_map = np.zeros((size, size), dtype=bool)
        self._min_height = min_height
        self._max_height = max_height
        self._area_thresh_in_pixels = area_thresh * (self.pixels_per_meter**2)
        self._hole_area_thresh = hole_area_thresh
        kernel_size = self.pixels_per_meter * agent_radius * 2
        # round kernel_size to nearest odd number
        kernel_size = int(kernel_size) + (int(kernel_size) % 2 == 0)
        self._navigable_kernel = np.ones((kernel_size, kernel_size), np.uint8)

    def reset(self) -> None:
        super().reset()
        self._navigable_map.fill(0)
        self.explored_area.fill(0)
        self.seen_map.fill(0)
        self._frontiers_px = np.array([])
        self.frontiers = np.array([])

    def update_map(
        self,
        depth: np.ndarray,
        camera_in: np.ndarray,
        camera_transform: np.ndarray,
        min_depth: float,
        max_depth: float,
        topdown_fov: float,
        verbose: bool = False,
        agent_path: str = './',
    ) -> None:
        """
        Adds all obstacles from the current view to the map. Also updates the area
        that the robot has explored so far.

        Args:
            depth (np.ndarray): The depth image to use for updating the object map. It
                is normalized to the range [0, 1] and has a shape of (height, width).

            tf_camera_to_episodic (np.ndarray): The transformation matrix from the
                camera to the episodic coordinate frame.
            min_depth (float): The minimum depth value (in meters) of the depth image.
            max_depth (float): The maximum depth value (in meters) of the depth image.
            fx (float): The focal length of the camera in the x direction.
            fy (float): The focal length of the camera in the y direction.
            topdown_fov (float): The field of view of the depth camera projected onto
                the topdown map.
            explore (bool): Whether to update the explored area.
            update_obstacles (bool): Whether to update the obstacle map.
        """
        # extract info from observations
        depth = filter_depth(depth, blur_type=None)
        camera_ex = get_extrinsic_matrix(camera_transform)

        # update obstacle map
        if self._hole_area_thresh == -1:
            filled_depth = depth.copy()
            filled_depth[depth == 0] = 1.0
        else:
            filled_depth = fill_small_holes(depth, max_depth, self._hole_area_thresh)
        mask = (depth < max_depth) * (depth > min_depth)
        point_cloud_episodic_frame = get_world_points_from_image_coords(depth, mask, camera_ex, camera_in)
        obstacle_cloud = filter_points_by_height(point_cloud_episodic_frame, self._min_height, self._max_height)

        # update seen map
        if len(point_cloud_episodic_frame) > 0:
            _xy_points = point_cloud_episodic_frame[:, :2]
            _pixel_points = self._xy_to_px(_xy_points)
            self.seen_map[_pixel_points[:, 1], _pixel_points[:, 0]] = 1
            self.seen_map = fill_small_holes(self.seen_map, 1, 100)

        # Populate topdown map with obstacle locations
        x = y = width = height = 0
        if len(obstacle_cloud) > 0:
            xy_points = obstacle_cloud[:, :2]
            pixel_points = self._xy_to_px(xy_points)
            self._map[pixel_points[:, 1], pixel_points[:, 0]] = 1

            x, y = np.min(pixel_points, axis=0)
            width, height = 1 + np.max(pixel_points, axis=0) - (x, y)

        # Update the navigable area, which is an inverse of the obstacle map after a
        # dilation operation to accommodate the robot's radius.
        self._navigable_map = 1 - cv2.dilate(
            self._map.astype(np.uint8),
            self._navigable_kernel,
            iterations=1,
        ).astype(bool)

        if verbose:
            plt.imsave(
                os.path.join(agent_path, 'images/seen_map.jpg'),
                self.seen_map * self._navigable_map,
            )
            plt.imsave(os.path.join(agent_path, 'images/obstacle_map.jpg'), self._map)
            plt.imsave(
                os.path.join(agent_path, 'images/navigatable_map.jpg'),
                self._navigable_map,
            )

        # Update the explored area
        camera_position, camera_rotation = extract_camera_pos_zyxrot(camera_transform)
        camera_xy_location = camera_position[:2].reshape(1, 2)
        agent_pixel_location = self._xy_to_px(camera_xy_location)[0]
        new_explored_area = reveal_fog_of_war(
            top_down_map=self._navigable_map.astype(np.uint8),
            current_fog_of_war_mask=np.zeros_like(self._map, dtype=np.uint8),
            current_point=agent_pixel_location[::-1],
            current_angle=-np.pi / 2 - camera_rotation[0],
            fov=np.rad2deg(topdown_fov),
            max_line_len=max_depth * self.pixels_per_meter,
        )
        new_explored_area = cv2.dilate(new_explored_area, np.ones((3, 3), np.uint8), iterations=1)
        self.explored_area[new_explored_area > 0] = 1
        self.explored_area[self._navigable_map == 0] = 0
        # contours, _ = cv2.findContours(
        #     self.explored_area.astype(np.uint8),
        #     cv2.RETR_EXTERNAL,
        #     cv2.CHAIN_APPROX_SIMPLE,
        # )
        # if len(contours) > 1:
        #     min_dist = np.inf
        #     best_idx = 0
        #     for idx, cnt in enumerate(contours):
        #         dist = cv2.pointPolygonTest(cnt, tuple([int(i) for i in agent_pixel_location]), True)
        #         if dist >= 0:
        #             best_idx = idx
        #             break
        #         elif abs(dist) < min_dist:
        #             min_dist = abs(dist)
        #             best_idx = idx
        #     new_area = np.zeros_like(self.explored_area, dtype=np.uint8)
        #     cv2.drawContours(new_area, contours, best_idx, 1, -1)  # type: ignore
        #     self.explored_area = new_area.astype(bool)

        # Compute frontier locations
        self._frontiers_px = self._get_frontiers()
        if verbose:
            explored_area_uint8 = self.explored_area.astype(np.uint8)
            for frontier in self._frontiers_px:
                cv2.circle(explored_area_uint8, tuple([int(i) for i in frontier]), 5, 1, -1)
            plt.imsave(
                os.path.join(agent_path, 'images/explored_with_frontiers.jpg'),
                explored_area_uint8,
            )
            plt.imsave(os.path.join(agent_path, 'images/explored_map.jpg'), self.explored_area)
        if len(self._frontiers_px) == 0:
            if self.seen_map.any():
                ys, xs = np.where(self.seen_map)
                coordinate = np.hstack((xs.reshape(-1, 1), ys.reshape(-1, 1)))
                frontier_idx = np.unique(np.concatenate(([0], np.argmax(squareform(pdist(coordinate)), axis=1))))
                self._frontiers_px = coordinate[frontier_idx[: max(10, len(frontier_idx))]]
                self.frontiers_px = self._px_to_xy(self._frontiers_px)
            else:
                self.frontiers = camera_position[:2].reshape(1, 2)
        else:
            self.frontiers = self._px_to_xy(self._frontiers_px)

        return {'x': x, 'y': y, 'width': width, 'height': height}

    def _get_frontiers(self) -> np.ndarray:
        """Returns the frontiers of the map."""
        # Dilate the explored area slightly to prevent small gaps between the explored
        # area and the unnavigable area from being detected as frontiers.
        explored_area = cv2.dilate(
            self.explored_area.astype(np.uint8),
            np.ones((5, 5), np.uint8),
            iterations=1,
        )
        frontiers = detect_frontier_waypoints(
            self._navigable_map.astype(np.uint8),
            explored_area,
            self._area_thresh_in_pixels,
        )
        if len(frontiers) == 0:
            frontiers = detect_frontier_waypoints(self._navigable_map.astype(np.uint8), explored_area)
        return frontiers

    def visualize(self) -> np.ndarray:
        """Visualizes the map."""
        vis_img = np.ones((*self._map.shape[:2], 3), dtype=np.uint8) * 255
        # Draw explored area in light green
        vis_img[self.explored_area == 1] = (200, 255, 200)
        # Draw unnavigable areas in gray
        vis_img[self._navigable_map == 0] = self.radius_padding_color
        # Draw obstacles in black
        vis_img[self._map == 1] = (0, 0, 0)
        # Draw frontiers in blue (200, 0, 0)
        for frontier in self._frontiers_px:
            cv2.circle(vis_img, tuple([int(i) for i in frontier]), 5, (200, 0, 0), 2)

        vis_img = cv2.flip(vis_img, 0)

        if len(self._camera_positions) > 0:
            self._traj_vis.draw_trajectory(
                vis_img,
                self._camera_positions,
                self._last_camera_yaw,
            )

        return vis_img


def filter_points_by_height(points: np.ndarray, min_height: float, max_height: float) -> np.ndarray:
    return points[(points[:, 2] >= min_height) & (points[:, 2] <= max_height)]
