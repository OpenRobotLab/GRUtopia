# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

from typing import Dict, Union

import cv2
import numpy as np
import open3d as o3d
from agent_utils.geometry_utils import (
    cam2world,
    extract_camera_pos_zyxrot,
    get_camera_pointcloud,
    get_extrinsic_matrix,
    within_fov_cone,
)


class ObjectPointCloudMap:
    clouds: Dict[str, np.ndarray] = {}
    use_dbscan: bool = True

    def __init__(self, erosion_size: float) -> None:
        self._erosion_size = erosion_size
        self.last_target_coord: Union[np.ndarray, None] = None

    def reset(self) -> None:
        self.clouds = {}
        self.last_target_coord = None

    def has_object(self, target_class: str) -> bool:
        return target_class in self.clouds and len(self.clouds[target_class]) > 0

    def update_map(
        self,
        rgb_img: np.ndarray,
        depth_img: np.ndarray,
        object_mask: np.ndarray,
        tf_camera_to_episodic: np.ndarray,
        camera_in: np.ndarray,
        min_depth: float,
        max_depth: float,
    ) -> None:
        """Updates the object map with the latest information from the agent."""
        local_cloud = self._extract_object_cloud(depth_img, object_mask, min_depth, max_depth, camera_in)
        if len(local_cloud) == 0:
            return

        # For second-class, bad detections that are too offset or out of range, we
        # assign a random number to the last column of its point cloud that can later
        # be used to identify which points came from the same detection.
        if too_offset(object_mask):
            within_range = np.ones_like(local_cloud[:, 0]) * np.random.rand()
        else:
            # Mark all points of local_cloud whose distance from the camera is too far
            # as being out of range
            within_range = (local_cloud[:, 0] <= max_depth * 0.95) * 1.0  # 5% margin
            # All values of 1 in within_range will be considered within range, and all
            # values of 0 will be considered out of range; these 0s need to be
            # assigned with a random number so that they can be identified later.
            within_range = within_range.astype(np.float32)
            within_range[within_range == 0] = np.random.rand()
        camera_ex = get_extrinsic_matrix(tf_camera_to_episodic)
        global_cloud = cam2world(local_cloud.T, camera_ex)
        global_cloud = np.concatenate((global_cloud, within_range[:, None]), axis=1)

        curr_position = tf_camera_to_episodic[:3, 3]
        closest_point = self._get_closest_point(global_cloud, curr_position)
        dist = np.linalg.norm(closest_point[:3] - curr_position)
        if dist < 1.0 or len(global_cloud) == 0:
            # Object is too close to trust as a valid object
            return

        self._update_object_clouds(rgb_img, object_mask, global_cloud)

    def get_best_object(self, target_id: int, curr_position: np.ndarray) -> np.ndarray:
        target_cloud = self.get_target_cloud(target_id)

        closest_point_2d = self._get_closest_point(target_cloud, curr_position)[:2]

        if self.last_target_coord is None:
            self.last_target_coord = closest_point_2d
        else:
            # Do NOT update self.last_target_coord if:
            # 1. the closest point is only slightly different
            # 2. the closest point is a little different, but the agent is too far for
            #    the difference to matter much
            delta_dist = np.linalg.norm(closest_point_2d - self.last_target_coord)
            if delta_dist < 0.1:
                # closest point is only slightly different
                return self.last_target_coord
            elif delta_dist < 0.5 and np.linalg.norm(curr_position - closest_point_2d) > 2.0:
                # closest point is a little different, but the agent is too far for
                # the difference to matter much
                return self.last_target_coord
            else:
                self.last_target_coord = closest_point_2d

        return self.last_target_coord

    def update_explored(self, tf_camera_to_episodic: np.ndarray, max_depth: float, cone_fov: float) -> None:
        """
        This method will remove all point clouds in self.clouds that were originally
        detected to be out-of-range, but are now within range. This is just a heuristic
        that suppresses ephemeral false positives that we now confirm are not actually
        target objects.

        Args:
            tf_camera_to_episodic: The transform from the camera to the episode frame.
            max_depth: The maximum distance from the camera that we consider to be
                within range.
            cone_fov: The field of view of the camera.
        """
        camera_coordinates, camera_rotation = extract_camera_pos_zyxrot(tf_camera_to_episodic)
        camera_yaw = camera_rotation[0]

        for obj in self.clouds:
            within_range = within_fov_cone(
                camera_coordinates,
                camera_yaw,
                cone_fov,
                max_depth * 0.5,
                self.clouds[obj]['clouds'],
            )
            range_ids = set(within_range[..., -1].tolist())
            for range_id in range_ids:
                if range_id == 1:
                    # Detection was originally within range
                    continue
                # Remove all points from self.clouds[obj] that have the same range_id
                self.clouds[obj]['clouds'] = self.clouds[obj]['clouds'][self.clouds[obj]['clouds'][..., -1] != range_id]

    def get_target_cloud(self, target_id: int) -> np.ndarray:
        target_cloud = self.clouds[target_id]['clouds'].copy()
        # Determine whether any points are within range
        within_range_exists = np.any(target_cloud[:, -1] == 1)
        if within_range_exists:
            # Filter out all points that are not within range
            target_cloud = target_cloud[target_cloud[:, -1] == 1]
        return target_cloud

    def _update_object_clouds(
        self,
        rgb: np.ndarray,
        obj_mask: np.ndarray,
        new_clouds: np.ndarray,
        proportion: float = 0.9,
    ):
        current_id = len(self.clouds)
        clouds_bbox = compute_bbox(new_clouds, proportion)
        new_flag = True
        for key, clouds_data in self.clouds.items():
            intersect = bbox_intersection(clouds_bbox, clouds_data['bbox'])
            if intersect:
                new_flag = False
                annotated_rgb = self._draw_annotated_image(key, rgb, obj_mask)
                self.clouds[key]['clouds'] = np.concatenate((clouds_data['clouds'], new_clouds), axis=0)
                self.clouds[key]['clouds'] = get_random_subarray(self.clouds[key]['clouds'], 5000)
                self.clouds[key]['bbox'] = compute_bbox(self.clouds[key]['clouds'], proportion)
                self.clouds[key]['images'].append({'number': new_clouds.shape[0], 'image': annotated_rgb})
        if new_flag:
            annotated_rgb = self._draw_annotated_image(current_id, rgb, obj_mask)
            self.clouds[current_id] = {
                'clouds': new_clouds,
                'images': [{'number': new_clouds.shape[0], 'image': annotated_rgb}],
                'bbox': clouds_bbox,
            }

    def _draw_annotated_image(self, current_id: int, rgb: np.ndarray, obj_mask: np.ndarray):
        contours, _ = cv2.findContours(obj_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # annotated_rgb = cv2.drawContours(rgb.copy(), contours, -1, (255, 0, 0), 2)

        # Find the contour with the largest area
        max_contour = max(contours, key=cv2.contourArea)

        # Calculate the centroid of the largest contour
        M = cv2.moments(max_contour)
        cx, cy = (
            int(M['m10'] / M['m00']),
            int(M['m01'] / M['m00'])
            if M['m00'] != 0
            else tuple(np.mean(np.argwhere(obj_mask), axis=0).squeeze())[::-1],
        )

        # Annotate the image with the current_id at the centroid (assume current_id is the number to be labeled)
        annotated_rgb = rgb.copy()  # Copy the original image to preserve it
        # cv2.drawContours(annotated_rgb, [max_contour], -1, (255, 0, 0), 2)
        cv2.putText(
            annotated_rgb,
            str(current_id),
            (cx, cy),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        return annotated_rgb

    def _extract_object_cloud(
        self,
        depth: np.ndarray,
        object_mask: np.ndarray,
        min_depth: float,
        max_depth: float,
        camera_in: np.ndarray,
    ) -> np.ndarray:
        valid_depth = depth.copy()
        valid_depth[valid_depth == 0] = max_depth  # set all holes (0) to just be far (1)

        mask = (depth < max_depth) * (depth > min_depth)
        final_mask = object_mask * 255
        final_mask = cv2.erode(final_mask, None, iterations=self._erosion_size)  # type: ignore
        final_mask = (final_mask * mask).astype(bool)

        cloud = get_camera_pointcloud(valid_depth, final_mask, camera_in).T
        cloud = get_random_subarray(cloud, 5000)
        if self.use_dbscan:
            cloud = open3d_dbscan_filtering(cloud)

        return cloud

    def _get_closest_point(self, cloud: np.ndarray, curr_position: np.ndarray) -> np.ndarray:
        ndim = curr_position.shape[0]
        if self.use_dbscan:
            # Return the point that is closest to curr_position, which is 2D
            closest_point = cloud[np.argmin(np.linalg.norm(cloud[:, :ndim] - curr_position, axis=1))]
        else:
            # Calculate the Euclidean distance from each point to the reference point
            if ndim == 2:
                ref_point = np.concatenate((curr_position, np.array([0.5])))
            else:
                ref_point = curr_position
            distances = np.linalg.norm(cloud[:, :3] - ref_point, axis=1)

            # Use argsort to get the indices that would sort the distances
            sorted_indices = np.argsort(distances)

            # Get the top 20% of points
            percent = 0.25
            top_percent = sorted_indices[: int(percent * len(cloud))]
            try:
                median_index = top_percent[int(len(top_percent) / 2)]
            except IndexError:
                median_index = 0
            closest_point = cloud[median_index]
        return closest_point


def compute_bbox(points: np.ndarray, proportion: float):
    num_points = points.shape[0]
    sorted_indices = np.argsort(np.linalg.norm(points - np.mean(points, axis=0), axis=1))
    selected_indices = sorted_indices[: int(num_points * proportion)]
    selected_points = points[selected_indices]
    min_bound_selected = np.min(selected_points, axis=0)
    max_bound_selected = np.max(selected_points, axis=0)
    return min_bound_selected, max_bound_selected


def bbox_intersection(bbox1, bbox2):
    min1, max1 = bbox1
    min2, max2 = bbox2

    min_intersection = np.maximum(min1, min2)
    max_intersection = np.minimum(max1, max2)

    if np.all(min_intersection <= max_intersection):
        return True
    else:
        return False


def open3d_dbscan_filtering(points: np.ndarray, eps: float = 0.8, min_points: int = 100) -> np.ndarray:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Perform DBSCAN clustering
    labels = np.array(pcd.cluster_dbscan(eps, min_points))

    # Count the points in each cluster
    unique_labels, label_counts = np.unique(labels, return_counts=True)

    # Exclude noise points, which are given the label -1
    non_noise_labels_mask = unique_labels != -1
    non_noise_labels = unique_labels[non_noise_labels_mask]
    non_noise_label_counts = label_counts[non_noise_labels_mask]

    if len(non_noise_labels) == 0:  # only noise was detected
        return np.array([])

    # Find the label of the largest non-noise cluster
    largest_cluster_label = non_noise_labels[np.argmax(non_noise_label_counts)]

    # Get the indices of points in the largest non-noise cluster
    largest_cluster_indices = np.where(labels == largest_cluster_label)[0]

    # Get the points in the largest non-noise cluster
    largest_cluster_points = points[largest_cluster_indices]

    return largest_cluster_points


def visualize_and_save_point_cloud(point_cloud: np.ndarray, save_path: str) -> None:
    """Visualizes an array of 3D points and saves the visualization as a PNG image.

    Args:
        point_cloud (np.ndarray): Array of 3D points with shape (N, 3).
        save_path (str): Path to save the PNG image.
    """
    import matplotlib.pyplot as plt

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x = point_cloud[:, 0]
    y = point_cloud[:, 1]
    z = point_cloud[:, 2]

    ax.scatter(x, y, z, c='b', marker='o')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.savefig(save_path)
    plt.close()


def get_random_subarray(points: np.ndarray, size: int) -> np.ndarray:
    """
    This function returns a subarray of a given 3D points array. The size of the
    subarray is specified by the user. The elements of the subarray are randomly
    selected from the original array. If the size of the original array is smaller than
    the specified size, the function will simply return the original array.

    Args:
        points (numpy array): A numpy array of 3D points. Each element of the array is a
            3D point represented as a numpy array of size 3.
        size (int): The desired size of the subarray.

    Returns:
        numpy array: A subarray of the original points array.
    """
    if len(points) <= size:
        return points
    indices = np.random.choice(len(points), size, replace=False)
    return points[indices]


def too_offset(mask: np.ndarray) -> bool:
    """
    This will return true if the entire bounding rectangle of the mask is either on the
    left or right third of the mask. This is used to determine if the object is too far
    to the side of the image to be a reliable detection.

    Args:
        mask (numpy array): A 2D numpy array of 0s and 1s representing the mask of the
            object.
    Returns:
        bool: True if the object is too offset, False otherwise.
    """
    # Find the bounding rectangle of the mask
    x, y, w, h = cv2.boundingRect(mask)

    # Calculate the thirds of the mask
    third = mask.shape[1] // 3

    # Check if the entire bounding rectangle is in the left or right third of the mask
    if x + w <= third:
        # Check if the leftmost point is at the edge of the image
        # return x == 0
        return x <= int(0.05 * mask.shape[1])
    elif x >= 2 * third:
        # Check if the rightmost point is at the edge of the image
        # return x + w == mask.shape[1]
        return x + w >= int(0.95 * mask.shape[1])
    else:
        return False
