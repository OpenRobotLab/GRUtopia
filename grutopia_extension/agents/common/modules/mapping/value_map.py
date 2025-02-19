# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import glob
import json
import os
import os.path as osp
import shutil
import time
import warnings
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
from agent_utils.geometry_utils import extract_camera_pos_zyxrot, get_rotation_matrix
from agent_utils.img_utils import (
    monochannel_to_inferno_rgb,
    pixel_value_within_radius,
    place_img_in_img,
    rotate_image,
)
from modules.mapping.base_map import BaseMap

DEBUG = False
SAVE_VISUALIZATIONS = False
RECORDING = os.environ.get('RECORD_VALUE_MAP', '0') == '1'
PLAYING = os.environ.get('PLAY_VALUE_MAP', '0') == '1'
RECORDING_DIR = './images/value_map_recordings'
JSON_PATH = osp.join(RECORDING_DIR, 'data.json')
KWARGS_JSON = osp.join(RECORDING_DIR, 'kwargs.json')


class ValueMap(BaseMap):
    """Generates a map representing how valuable explored regions of the environment
    are with respect to finding and navigating to the target object."""

    _confidence_masks: Dict[Tuple[float, float], np.ndarray] = {}
    _camera_positions: List[np.ndarray] = []
    _last_camera_yaw: float = 0.0
    _min_confidence: float = 0.25
    _decision_threshold: float = 0.35
    _map: np.ndarray

    def __init__(
        self,
        value_channels: int,
        pixels_per_meter: int = 20,
        size: int = 1000,
        use_max_confidence: bool = True,
        fusion_type: str = 'default',
        obstacle_map: Optional['ObstacleMap'] = None,  # type: ignore # noqa: F821
    ) -> None:
        """
        Args:
            value_channels: The number of channels in the value map.
            size: The size of the value map in pixels.
            use_max_confidence: Whether to use the maximum confidence value in the value
                map or a weighted average confidence value.
            fusion_type: The type of fusion to use when combining the value map with the
                obstacle map.
            obstacle_map: An optional obstacle map to use for overriding the occluded
                areas of the FOV
        """
        if PLAYING:
            size = 2000
        super().__init__(size, pixels_per_meter)
        self._value_map = np.zeros((size, size, value_channels), np.float32)
        self._value_channels = value_channels
        self._use_max_confidence = use_max_confidence
        self._fusion_type = fusion_type
        self._obstacle_map = obstacle_map
        if self._obstacle_map is not None:
            assert self._obstacle_map.pixels_per_meter == self.pixels_per_meter
            assert self._obstacle_map.size == self.size
        if os.environ.get('MAP_FUSION_TYPE', '') != '':
            self._fusion_type = os.environ['MAP_FUSION_TYPE']

        if RECORDING:
            if osp.isdir(RECORDING_DIR):
                warnings.warn(f'Recording directory {RECORDING_DIR} already exists. Deleting it.')
                shutil.rmtree(RECORDING_DIR)
            os.mkdir(RECORDING_DIR)
            # Dump all args to a file
            with open(KWARGS_JSON, 'w') as f:
                json.dump(
                    {
                        'value_channels': value_channels,
                        'size': size,
                        'use_max_confidence': use_max_confidence,
                    },
                    f,
                )
            # Create a blank .json file inside for now
            with open(JSON_PATH, 'w') as f:
                f.write('{}')

    def reset(self) -> None:
        super().reset()
        self._value_map.fill(0)

    def update_map(
        self,
        values: np.ndarray,
        depth: np.ndarray,
        tf_camera_to_episodic: np.ndarray,
        min_depth: float,
        max_depth: float,
        fov: float,
    ) -> None:
        """Updates the value map with the given depth image, pose, and value to use.

        Args:
            values: The value to use for updating the map.
            depth: The depth image to use for updating the map; expected to be already
                normalized to the range [0, 1].
            tf_camera_to_episodic: The transformation matrix from the episodic frame to
                the camera frame.
            min_depth: The minimum depth value in meters.
            max_depth: The maximum depth value in meters.
            fov: The field of view of the camera in RADIANS.
        """
        assert (
            len(values) == self._value_channels
        ), f'Incorrect number of values given ({len(values)}). Expected {self._value_channels}.'

        curr_map = self._localize_new_data(depth, tf_camera_to_episodic, min_depth, max_depth, fov)

        # Fuse the new data with the existing data
        self._fuse_new_data(curr_map, values)

        if RECORDING:
            idx = len(glob.glob(osp.join(RECORDING_DIR, '*.png')))
            img_path = osp.join(RECORDING_DIR, f'{idx:04d}.png')
            cv2.imwrite(img_path, (depth * 255).astype(np.uint8))
            with open(JSON_PATH, 'r') as f:
                data = json.load(f)
            data[img_path] = {
                'values': values.tolist(),
                'tf_camera_to_episodic': tf_camera_to_episodic.tolist(),
                'min_depth': min_depth,
                'max_depth': max_depth,
                'fov': fov,
            }
            with open(JSON_PATH, 'w') as f:
                json.dump(data, f)

    def sort_waypoints(
        self, waypoints: np.ndarray, radius: float, reduce_fn: Optional[Callable] = None
    ) -> Tuple[np.ndarray, List[float]]:
        """Selects the best waypoint from the given list of waypoints.

        Args:
            waypoints (np.ndarray): An array of 2D waypoints to choose from.
            radius (float): The radius in meters to use for selecting the best waypoint.
            reduce_fn (Callable, optional): The function to use for reducing the values
                within the given radius. Defaults to np.max.

        Returns:
            Tuple[np.ndarray, List[float]]: A tuple of the sorted waypoints and
                their corresponding values.
        """
        radius_px = int(radius * self.pixels_per_meter)

        def get_value(point: np.ndarray) -> Union[float, Tuple[float, ...]]:
            x, y = point
            px = int(-x * self.pixels_per_meter) + self._episode_pixel_origin[0]
            py = int(-y * self.pixels_per_meter) + self._episode_pixel_origin[1]
            point_px = (self._value_map.shape[0] - px, py)
            all_values = [
                pixel_value_within_radius(self._value_map[..., c], point_px, radius_px)
                for c in range(self._value_channels)
            ]
            if len(all_values) == 1:
                return all_values[0]
            return tuple(all_values)

        values = [get_value(point) for point in waypoints]

        if self._value_channels > 1:
            assert reduce_fn is not None, 'Must provide a reduction function when using multiple value channels.'
            values = reduce_fn(values)

        # Use np.argsort to get the indices of the sorted values
        sorted_inds = np.argsort([-v for v in values])  # type: ignore
        sorted_values = [values[i] for i in sorted_inds]
        sorted_frontiers = np.array([waypoints[i] for i in sorted_inds])

        return sorted_frontiers, sorted_values

    def visualize(
        self,
        markers: Optional[List[Tuple[np.ndarray, Dict[str, Any]]]] = None,
        reduce_fn: Callable = lambda i: np.max(i, axis=-1),
        obstacle_map: Optional['ObstacleMap'] = None,  # type: ignore # noqa: F821
    ) -> np.ndarray:
        """Return an image representation of the map"""
        # Must negate the y values to get the correct orientation
        reduced_map = reduce_fn(self._value_map).copy()
        if obstacle_map is not None:
            reduced_map[obstacle_map.explored_area == 0] = 0
        map_img = np.flipud(reduced_map)
        # Make all 0s in the value map equal to the max value, so they don't throw off
        # the color mapping (will revert later)
        zero_mask = map_img == 0
        map_img[zero_mask] = np.max(map_img)
        map_img = monochannel_to_inferno_rgb(map_img)
        # Revert all values that were originally zero to white
        map_img[zero_mask] = (255, 255, 255)
        if len(self._camera_positions) > 0:
            self._traj_vis.draw_trajectory(
                map_img,
                self._camera_positions,
                self._last_camera_yaw,
            )

            if markers is not None:
                for pos, marker_kwargs in markers:
                    map_img = self._traj_vis.draw_circle(map_img, pos, **marker_kwargs)

        return map_img

    def _process_local_data(self, depth: np.ndarray, fov: float, min_depth: float, max_depth: float) -> np.ndarray:
        """Using the FOV and depth, return the visible portion of the FOV.

        Args:
            depth: The depth image to use for determining the visible portion of the
                FOV.
        Returns:
            A mask of the visible portion of the FOV.
        """
        # Squeeze out the channel dimension if depth is a 3D array
        if len(depth.shape) == 3:
            depth = depth.squeeze(2)
        # Squash depth image into one row with the max depth value for each column
        depth_row = np.max(depth, axis=0)

        # Create a linspace of the same length as the depth row from -fov/2 to fov/2
        angles = np.linspace(-fov / 2, fov / 2, len(depth_row))

        # Assign each value in the row with an x, y coordinate depending on 'angles'
        # and the max depth value for that column
        x = depth_row
        y = depth_row * np.tan(angles)

        # Get blank cone mask
        cone_mask = self._get_confidence_mask(fov, max_depth)

        # Convert the x, y coordinates to pixel coordinates
        x = (x * self.pixels_per_meter + cone_mask.shape[0] / 2).astype(int)
        y = (y * self.pixels_per_meter + cone_mask.shape[1] / 2).astype(int)

        # Create a contour from the x, y coordinates, with the top left and right
        # corners of the image as the first two points
        last_row = cone_mask.shape[0] - 1
        last_col = cone_mask.shape[1] - 1
        start = np.array([[0, last_col]])
        end = np.array([[last_row, last_col]])
        contour = np.concatenate((start, np.stack((y, x), axis=1), end), axis=0)

        # Draw the contour onto the cone mask, in filled-in black
        visible_mask = cv2.drawContours(cone_mask, [contour], -1, 0, -1)  # type: ignore

        if DEBUG:
            vis = cv2.cvtColor((cone_mask * 255).astype(np.uint8), cv2.COLOR_GRAY2RGB)
            cv2.drawContours(vis, [contour], -1, (0, 0, 255), -1)
            for point in contour:
                vis[point[1], point[0]] = (0, 255, 0)
            if SAVE_VISUALIZATIONS:
                # Create visualizations directory if it doesn't exist
                if not os.path.exists(RECORDING_DIR):
                    os.makedirs(RECORDING_DIR)
                # Expand the depth_row back into a full image
                depth_row_full = np.repeat(depth_row.reshape(1, -1), depth.shape[0], axis=0)
                # Stack the depth images with the visible mask
                depth_rgb = cv2.cvtColor((depth * 255).astype(np.uint8), cv2.COLOR_GRAY2RGB)
                depth_row_full = cv2.cvtColor((depth_row_full * 255).astype(np.uint8), cv2.COLOR_GRAY2RGB)
                vis = np.flipud(vis)
                new_width = int(vis.shape[1] * (depth_rgb.shape[0] / vis.shape[0]))
                vis_resized = cv2.resize(vis, (new_width, depth_rgb.shape[0]))
                vis = np.hstack((depth_rgb, depth_row_full, vis_resized))
                time_id = int(time.time() * 1000)
                cv2.imwrite(os.path.join(RECORDING_DIR, f'{time_id}.png'), vis)
            else:
                cv2.imshow('obstacle mask', vis)
                cv2.waitKey(0)

        return visible_mask

    def _localize_new_data(
        self,
        depth: np.ndarray,
        camera_transorm: np.ndarray,
        min_depth: float,
        max_depth: float,
        fov: float,
    ) -> np.ndarray:
        # Get new portion of the map
        curr_data = self._process_local_data(depth, fov, min_depth, max_depth)

        # Rotate this new data to match the camera's orientation
        camera_position, camera_rotation = extract_camera_pos_zyxrot(camera_transorm)

        yaw = camera_rotation[0]
        if PLAYING:
            if yaw > 0:
                yaw = 0
            else:
                yaw = np.deg2rad(30)
        curr_data = rotate_image(curr_data, -np.pi / 2 - yaw)

        # Determine where this mask should be overlaid
        cam_x, cam_y = camera_position[:2]

        # Convert to pixel units
        px = int(cam_x * self.pixels_per_meter) + self._episode_pixel_origin[0]
        py = int(-cam_y * self.pixels_per_meter) + self._episode_pixel_origin[1]

        # Overlay the new data onto the map
        curr_map = np.zeros_like(self._map)
        curr_map = place_img_in_img(curr_map, curr_data, px, py)

        return curr_map

    def _get_blank_cone_mask(self, fov: float, max_depth: float) -> np.ndarray:
        """Generate a FOV cone without any obstacles considered"""
        size = int(max_depth * self.pixels_per_meter)
        cone_mask = np.zeros((size * 2 + 1, size * 2 + 1))
        cone_mask = cv2.ellipse(  # type: ignore
            cone_mask,
            (size, size),  # center_pixel
            (size, size),  # axes lengths
            0,  # angle circle is rotated
            -np.rad2deg(fov) / 2 + 90,  # start_angle
            np.rad2deg(fov) / 2 + 90,  # end_angle
            1,  # color
            -1,  # thickness
        )
        return cone_mask

    def _get_confidence_mask(self, fov: float, max_depth: float) -> np.ndarray:
        """Generate a FOV cone with central values weighted more heavily"""
        if (fov, max_depth) in self._confidence_masks:
            return self._confidence_masks[(fov, max_depth)].copy()
        cone_mask = self._get_blank_cone_mask(fov, max_depth)
        adjusted_mask = np.zeros_like(cone_mask).astype(np.float32)
        for row in range(adjusted_mask.shape[0]):
            for col in range(adjusted_mask.shape[1]):
                horizontal = abs(row - adjusted_mask.shape[0] // 2)
                vertical = abs(col - adjusted_mask.shape[1] // 2)
                angle = np.arctan2(vertical, horizontal)
                angle = remap(angle, 0, fov / 2, 0, np.pi / 2)
                confidence = np.cos(angle) ** 2
                confidence = remap(confidence, 0, 1, self._min_confidence, 1)
                adjusted_mask[row, col] = confidence
        adjusted_mask = adjusted_mask * cone_mask
        self._confidence_masks[(fov, max_depth)] = adjusted_mask.copy()

        return adjusted_mask

    def _fuse_new_data(self, new_map: np.ndarray, values: np.ndarray) -> None:
        """Fuse the new data with the existing value and confidence maps.

        Args:
            new_map: The new new_map map data to fuse. Confidences are between
                0 and 1, with 1 being the most confident.
            values: The values attributed to the new portion of the map.
        """
        assert (
            len(values) == self._value_channels
        ), f'Incorrect number of values given ({len(values)}). Expected {self._value_channels}.'

        if self._obstacle_map is not None:
            # If an obstacle map is provided, we will use it to mask out the
            # new map
            explored_area = self._obstacle_map.explored_area
            new_map[explored_area == 0] = 0
            self._map[explored_area == 0] = 0
            self._value_map[explored_area == 0] *= 0

        if self._fusion_type == 'replace':
            # Ablation. The values from the current observation will overwrite any
            # existing values
            print('VALUE MAP ABLATION:', self._fusion_type)
            new_value_map = np.zeros_like(self._value_map)
            new_value_map[new_map > 0] = values
            self._map[new_map > 0] = new_map[new_map > 0]
            self._value_map[new_map > 0] = new_value_map[new_map > 0]
            return
        elif self._fusion_type == 'equal_weighting':
            # Ablation. Updated values will always be the mean of the current and
            # new values, meaning that confidence scores are forced to be the same.
            print('VALUE MAP ABLATION:', self._fusion_type)
            self._map[self._map > 0] = 1
            new_map[new_map > 0] = 1
        else:
            assert self._fusion_type == 'default', f'Unknown fusion type {self._fusion_type}'

        # Any values in the given map that are less confident than
        # self._decision_threshold AND less than the new_map in the existing map
        # will be silenced into 0s
        new_map_mask = np.logical_and(new_map < self._decision_threshold, new_map < self._map)
        new_map[new_map_mask] = 0

        if self._use_max_confidence:
            # For every pixel that has a higher new_map in the new map than the
            # existing value map, replace the value in the existing value map with
            # the new value
            higher_new_map_mask = new_map > self._map
            self._value_map[higher_new_map_mask] = values
            # Update the new_map map with the new new_map values
            self._map[higher_new_map_mask] = new_map[higher_new_map_mask]
        else:
            # Each pixel in the existing value map will be updated with a weighted
            # average of the existing value and the new value. The weight of each value
            # is determined by the current and new new_map values. The new_map map
            # will also be updated with using a weighted average in a similar manner.
            confidence_denominator = self._map + new_map
            with warnings.catch_warnings():
                warnings.filterwarnings('ignore', category=RuntimeWarning)
                weight_1 = self._map / confidence_denominator
                weight_2 = new_map / confidence_denominator

            weight_1_channeled = np.repeat(np.expand_dims(weight_1, axis=2), self._value_channels, axis=2)
            weight_2_channeled = np.repeat(np.expand_dims(weight_2, axis=2), self._value_channels, axis=2)

            self._value_map = self._value_map * weight_1_channeled + values * weight_2_channeled
            self._map = self._map * weight_1 + new_map * weight_2

            # Because confidence_denominator can have 0 values, any nans in either the
            # value or confidence maps will be replaced with 0
            self._value_map = np.nan_to_num(self._value_map)
            self._map = np.nan_to_num(self._map)


def remap(value: float, from_low: float, from_high: float, to_low: float, to_high: float) -> float:
    """Maps a value from one range to another.

    Args:
        value (float): The value to be mapped.
        from_low (float): The lower bound of the input range.
        from_high (float): The upper bound of the input range.
        to_low (float): The lower bound of the output range.
        to_high (float): The upper bound of the output range.

    Returns:
        float: The mapped value.
    """
    return (value - from_low) * (to_high - to_low) / (from_high - from_low) + to_low


def replay_from_dir() -> None:
    with open(KWARGS_JSON, 'r') as f:
        kwargs = json.load(f)
    with open(JSON_PATH, 'r') as f:
        data = json.load(f)

    v = ValueMap(**kwargs)

    sorted_keys = sorted(list(data.keys()))

    for img_path in sorted_keys:
        tf_camera_to_episodic = np.array(data[img_path]['tf_camera_to_episodic'])
        values = np.array(data[img_path]['values'])
        depth = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE).astype(np.float32) / 255.0
        v.update_map(
            values,
            depth,
            tf_camera_to_episodic,
            float(data[img_path]['min_depth']),
            float(data[img_path]['max_depth']),
            float(data[img_path]['fov']),
        )

        img = v.visualize()
        cv2.imshow('img', img)
        key = cv2.waitKey(0)
        if key == ord('q'):
            break


if __name__ == '__main__':
    if PLAYING:
        replay_from_dir()
        quit()

    v = ValueMap(value_channels=1)
    depth = cv2.imread('depth.png', cv2.IMREAD_GRAYSCALE).astype(np.float32) / 255.0
    img = v._process_local_data(
        depth=depth,
        fov=np.deg2rad(79),
        min_depth=0.5,
        max_depth=5.0,
    )
    cv2.imshow('img', (img * 255).astype(np.uint8))
    cv2.waitKey(0)

    num_points = 20

    x = [0, 10, 10, 0]
    y = [0, 0, 10, 10]
    angles = [0, np.pi / 2, np.pi, 3 * np.pi / 2]

    points = np.stack((x, y), axis=1)

    for pt, angle in zip(points, angles):
        tf = np.eye(4)
        tf[:2, 3] = pt
        tf[:2, :2] = get_rotation_matrix(angle)
        v.update_map(
            np.array([1]),
            depth,
            tf,
            min_depth=0.5,
            max_depth=5.0,
            fov=np.deg2rad(79),
        )
        img = v.visualize()
        cv2.imshow('img', img)
        key = cv2.waitKey(0)
        if key == ord('q'):
            break
