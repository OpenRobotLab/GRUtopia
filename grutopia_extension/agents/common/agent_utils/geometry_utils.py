# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

import math
from typing import Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R


def rho_theta(curr_pos: np.ndarray, curr_heading: float, curr_goal: np.ndarray) -> Tuple[float, float]:
    """Calculates polar coordinates (rho, theta) relative to a given position and
    heading to a given goal position. 'rho' is the distance from the agent to the goal,
    and theta is how many radians the agent must turn (to the left, CCW from above) to
    face the goal. Coordinates are in (x, y), where x is the distance forward/backwards,
    and y is the distance to the left or right (right is negative)

    Args:
        curr_pos (np.ndarray): Array of shape (2,) representing the current position.
        curr_heading (float): The current heading, in radians. It represents how many
            radians  the agent must turn to the left (CCW from above) from its initial
            heading to reach its current heading.
        curr_goal (np.ndarray): Array of shape (2,) representing the goal position.

    Returns:
        Tuple[float, float]: A tuple of floats representing the polar coordinates
            (rho, theta).
    """
    rotation_matrix = get_rotation_matrix(-curr_heading, ndims=2)
    local_goal = curr_goal - curr_pos
    local_goal = rotation_matrix @ local_goal

    rho = np.linalg.norm(local_goal)
    theta = np.arctan2(local_goal[1], local_goal[0])

    return float(rho), float(theta)


def get_rotation_matrix(angle: float, ndims: int = 2) -> np.ndarray:
    """Returns a 2x2 or 3x3 rotation matrix for a given angle; if 3x3, the z-axis is
    rotated."""
    if ndims == 2:
        return np.array(
            [
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle), np.cos(angle)],
            ]
        )
    elif ndims == 3:
        return np.array(
            [
                [np.cos(angle), -np.sin(angle), 0],
                [np.sin(angle), np.cos(angle), 0],
                [0, 0, 1],
            ]
        )
    else:
        raise ValueError('ndims must be 2 or 3')


def wrap_heading(theta: float) -> float:
    """Wraps given angle to be between -pi and pi.

    Args:
        theta (float): The angle in radians.
    Returns:
        float: The wrapped angle in radians.
    """
    return (theta + np.pi) % (2 * np.pi) - np.pi


def calculate_vfov(hfov: float, width: int, height: int) -> float:
    """Calculates the vertical field of view (VFOV) based on the horizontal field of
    view (HFOV), width, and height of the image sensor.

    Args:
        hfov (float): The HFOV in radians.
        width (int): Width of the image sensor in pixels.
        height (int): Height of the image sensor in pixels.

    Returns:
        A float representing the VFOV in radians.
    """
    # Calculate the diagonal field of view (DFOV)
    dfov = 2 * math.atan(math.tan(hfov / 2) * math.sqrt((width**2 + height**2) / (width**2 + height**2)))

    # Calculate the vertical field of view (VFOV)
    vfov = 2 * math.atan(math.tan(dfov / 2) * (height / math.sqrt(width**2 + height**2)))

    return vfov


def within_fov_cone(
    cone_origin: np.ndarray,
    cone_angle: float,
    cone_fov: float,
    cone_range: float,
    points: np.ndarray,
) -> np.ndarray:
    """Checks if points are within a cone of a given origin, angle, fov, and range.

    Args:
        cone_origin (np.ndarray): The origin of the cone.
        cone_angle (float): The angle of the cone in radians.
        cone_fov (float): The field of view of the cone in radians.
        cone_range (float): The range of the cone.
        points (np.ndarray): The points to check.

    Returns:
        np.ndarray: The subarray of points that are within the cone.
    """
    directions = points[:, :3] - cone_origin
    dists = np.linalg.norm(directions, axis=1)
    angles = np.arctan2(directions[:, 1], directions[:, 0])
    angle_diffs = np.mod(angles - cone_angle + np.pi, 2 * np.pi) - np.pi

    mask = np.logical_and(dists <= cone_range, np.abs(angle_diffs) <= cone_fov / 2)
    return points[mask]


def convert_to_global_frame(agent_pos: np.ndarray, agent_yaw: float, local_pos: np.ndarray) -> np.ndarray:
    """Converts a given position from the agent's local frame to the global frame.

    Args:
        agent_pos (np.ndarray): A 3D vector representing the agent's position in their
            local frame.
        agent_yaw (float): The agent's yaw in radians.
        local_pos (np.ndarray): A 3D vector representing the position to be converted in
            the agent's local frame.

    Returns:
        A 3D numpy array representing the position in the global frame.
    """
    # Append a homogeneous coordinate of 1 to the local position vector
    local_pos_homogeneous = np.append(local_pos, 1)

    # Construct the homogeneous transformation matrix
    transformation_matrix = xyz_yaw_to_tf_matrix(agent_pos, agent_yaw)

    # Perform the transformation using matrix multiplication
    global_pos_homogeneous = transformation_matrix.dot(local_pos_homogeneous)
    global_pos_homogeneous = global_pos_homogeneous[:3] / global_pos_homogeneous[-1]

    return global_pos_homogeneous


def extract_camera_pos_zyxrot(camera_transform: np.ndarray) -> float:
    """Extract the yaw angle from a 4x4 transformation matrix.

    Args:
        matrix (np.ndarray): A 4x4 transformation matrix.
    Returns:
        float: The yaw angle in radians.
    """
    assert camera_transform.shape == (4, 4), 'The input matrix must be 4x4'
    camera_transform = np.linalg.inv(camera_transform)
    camera_position = camera_transform[3, :3]
    camera_rotation = R.from_matrix(camera_transform[:3, :3].T).as_euler('ZYX')
    return camera_position, camera_rotation


def xyz_yaw_to_tf_matrix(xyz: np.ndarray, camera_orientation: np.ndarray) -> np.ndarray:
    """Converts a given position and yaw angle to a 4x4 transformation matrix.

    Args:
        xyz (np.ndarray): A 3D vector representing the position.
        yaw (float): The yaw angle in radians.
    Returns:
        np.ndarray: A 4x4 transformation matrix.
    """
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, 3] = xyz
    transformation_matrix[:3, :3] = R.from_quat(camera_orientation).as_matrix()
    return transformation_matrix


def closest_point_within_threshold(points_array: np.ndarray, target_point: np.ndarray, threshold: float) -> int:
    """Find the point within the threshold distance that is closest to the target_point.

    Args:
        points_array (np.ndarray): An array of 2D points, where each point is a tuple
            (x, y).
        target_point (np.ndarray): The target 2D point (x, y).
        threshold (float): The maximum distance threshold.

    Returns:
        int: The index of the closest point within the threshold distance.
    """
    distances = np.sqrt((points_array[:, 0] - target_point[0]) ** 2 + (points_array[:, 1] - target_point[1]) ** 2)
    within_threshold = distances <= threshold

    if np.any(within_threshold):
        closest_index = np.argmin(distances)
        return int(closest_index)

    return -1


def transform_points(transformation_matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    # Add a homogeneous coordinate of 1 to each point for matrix multiplication
    homogeneous_points = np.hstack((points, np.ones((points.shape[0], 1))))

    # Apply the transformation matrix to the points
    transformed_points = np.dot(transformation_matrix, homogeneous_points.T).T

    # Remove the added homogeneous coordinate and divide by the last coordinate
    return transformed_points[:, :3] / transformed_points[:, 3:]


def get_intrinsic_matrix(camera_params: dict):
    width, height = camera_params['renderProductResolution']
    focal_length = camera_params['cameraFocalLength'] / 10.0
    horizontal_aperture, _ = camera_params['cameraAperture'] / 10.0
    vertical_aperture = horizontal_aperture * (float(height) / width)
    fx = width * focal_length / horizontal_aperture
    fy = height * focal_length / vertical_aperture
    cx = width * 0.5
    cy = height * 0.5
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])


def get_camera_pointcloud(depth_image: np.ndarray, mask: np.ndarray, intrinsic_matrix: np.ndarray) -> np.ndarray:
    """Calculates the 3D coordinates (x, y, z) of points in the depth image based on
    the horizontal field of view (HFOV), the image width and height, the depth values,
    and the pixel x and y coordinates.

    Args:
        depth_image (np.ndarray): 2D depth image.
        mask (np.ndarray): 2D binary mask identifying relevant pixels.
        fx (float): Focal length in the x direction.
        fy (float): Focal length in the y direction.

    Returns:
        np.ndarray: Array of 3D coordinates (x, y, z) of the points in the image plane.
    """
    v, u = np.where(mask)
    depth = depth_image[v, u]
    points_2d = np.hstack((u.reshape(-1, 1), v.reshape(-1, 1)))
    homogeneous = np.pad(points_2d, ((0, 0), (0, 1)), constant_values=1.0)

    intrinsics_matrix_inv = np.linalg.inv(intrinsic_matrix)

    points_in_camera_axes = np.dot(intrinsics_matrix_inv, homogeneous.T * np.expand_dims(depth, 0))

    return points_in_camera_axes


def get_extrinsic_matrix(view_transform: np.ndarray):
    view_transform = np.linalg.inv(view_transform)
    extrinsic_matrix = np.dot(
        np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]),
        np.linalg.inv(view_transform.T),
    )
    return extrinsic_matrix


def cam2world(cam_pointcloud: np.ndarray, extrinsic_matrix: np.ndarray):
    view_matrix_ros_inv = np.linalg.inv(extrinsic_matrix)
    points_in_camera_axes_homogenous = np.pad(cam_pointcloud, ((0, 1), (0, 0)), constant_values=1.0)
    points_in_world_frame_homogenous = np.dot(view_matrix_ros_inv, points_in_camera_axes_homogenous)
    points_in_world_frame = points_in_world_frame_homogenous[:3, :].T

    return points_in_world_frame


def get_world_points_from_image_coords(
    depth: np.ndarray, mask: np.ndarray, extrinsic_matrix, intrinsic_matrix: np.ndarray
):
    """Using pinhole perspective projection, this method does the inverse projection given the depth of the
        pixels

    Args:
        points_2d (np.ndarray): 2d points (u, v) corresponds to the pixel coordinates. shape is (n, 2) where n is the number of points.
        depth (np.ndarray): depth corresponds to each of the pixel coords. shape is (n,)

    Returns:
        np.ndarray: (n, 3) 3d points (X, Y, Z) in world frame. shape is (n, 3) where n is the number of points.
    """
    points_in_camera_axes = get_camera_pointcloud(depth, mask, intrinsic_matrix)
    points_in_world_frame = cam2world(points_in_camera_axes, extrinsic_matrix)
    return points_in_world_frame


def get_fov(focal_length: float, image_height_or_width: int) -> float:
    """
    Given an fx and the image width, or an fy and the image height, returns the
    horizontal or vertical field of view, respectively.

    Args:
        focal_length: Focal length of the camera.
        image_height_or_width: Height or width of the image.

    Returns:
        Field of view in radians.
    """

    # Calculate the field of view using the formula
    fov = 2 * math.atan((image_height_or_width / 2) / focal_length)
    return fov


def pt_from_rho_theta(rho: float, theta: float) -> np.ndarray:
    """
    Given a rho and theta, computes the x and y coordinates.

    Args:
        rho: Distance from the origin.
        theta: Angle from the x-axis.

    Returns:
        numpy.ndarray: x and y coordinates.
    """
    x = rho * math.cos(theta)
    y = rho * math.sin(theta)

    return np.array([x, y])
