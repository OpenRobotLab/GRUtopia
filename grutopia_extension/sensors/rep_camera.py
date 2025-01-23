from collections import defaultdict
from typing import Dict

import numpy as np
import omni.replicator.core as rep
from pxr import Usd, UsdGeom

from grutopia.core.robot.robot import BaseRobot, Scene
from grutopia.core.robot.sensor import BaseSensor
from grutopia.core.util import log
from grutopia_extension.config.sensors import RepCameraModel


@BaseSensor.register('RepCamera')
class RepCamera(BaseSensor):
    """
    wrap of replicator render_product
    """

    def __init__(self, config: RepCameraModel, robot: BaseRobot, name: str = None, scene: Scene = None):
        super().__init__(config, robot, scene)
        self.size = None
        self.name = name
        self.config = config
        self.camera_prim_path = self.create_camera()
        self.rp = None
        self.rp_annotators = {}

    def init(self) -> None:
        """
        Initialize the camera sensor.

        TODO Add `camera frame config` into GRUtopia config file.
        """
        if self.rp is None:
            self.rp = rep.create.render_product(self.camera_prim_path, self.size)

    def create_camera(self) -> str:
        """Create an isaac-sim camera object.

        Initializes the camera's resolution and prim path based on configuration.

        Returns:
            i_Camera: The initialized camera object.
        """
        # Initialize the default resolution for the camera
        self.size = (64, 64)
        # Use the configured camera size if provided.
        if self.config.size is not None:
            self.size = self.config.size

        prim_path = self._robot.robot_model.prim_path + '/' + self.config.prim_path
        log.debug('camera_prim_path: ' + prim_path)
        log.debug('name            : ' + self.config.name)
        log.debug(f'size            : {self.size}')
        return prim_path

    def sensor_init(self) -> None:
        """
        Initialize the camera sensor.
        """
        # if self.config.enable:
        #     self._camera.initialize()
        #     self._camera.add_distance_to_image_plane_to_frame()
        pass

    def get_camera_data(self, data_names):
        """
        Get specified data from a camera.

        Parameters:
            data_names: list, a list of desired data names, can be any combination of "landmarks", "rgba", "depth", "pointcloud", "camera_params"

        Returns:
            output_data: dict, a dict of data corresponding to the requested data names
        """
        # ================== run once ================
        if 'landmarks' in data_names and 'landmarks' not in self.rp_annotators:
            bbox_2d_tight = rep.AnnotatorRegistry.get_annotator('bounding_box_2d_tight')
            bbox_2d_tight.attach(self.rp)
            self.rp_annotators['landmarks'] = bbox_2d_tight

        if 'rgba' in data_names and 'rgba' not in self.rp_annotators:
            ldr_color = rep.AnnotatorRegistry.get_annotator('LdrColor')
            ldr_color.attach(self.rp)
            self.rp_annotators['rgba'] = ldr_color

        if 'depth' in data_names and 'depth' not in self.rp_annotators:
            distance_to_image_plane = rep.AnnotatorRegistry.get_annotator('distance_to_image_plane')
            distance_to_image_plane.attach(self.rp)
            self.rp_annotators['depth'] = distance_to_image_plane

        if 'camera_params' in data_names and 'camera_params' not in self.rp_annotators:
            camera_params = rep.annotators.get('CameraParams').attach(self.rp)
            self.rp_annotators['camera_params'] = camera_params

        output_data = {}

        for name, annotator in self.rp_annotators.items():
            if name == 'landmarks':
                output_data['bounding_box_2d_tight'] = annotator.get_data()
                if len(output_data['bounding_box_2d_tight']['data']) > 0:
                    output_data['landmarks'] = self._get_face_to_instances(
                        output_data['bounding_box_2d_tight']['data'],
                        output_data['bounding_box_2d_tight']['info']['idToLabels'],
                    )
                else:
                    output_data['landmarks'] = []
                continue
            output_data[name] = annotator.get_data()

        if 'pointcloud' in data_names:
            try:
                output_data['pointcloud'] = self.get_pointcloud(output_data['depth'], output_data['camera_params'])
            except Exception:
                output_data['pointcloud'] = None

        return output_data

    # ============================================================================================================
    @staticmethod
    def as_type(data, dtype):
        if dtype == 'float32':
            return data.astype(np.float32)
        elif dtype == 'bool':
            return data.astype(bool)
        elif dtype == 'int32':
            return data.astype(np.int32)
        elif dtype == 'int64':
            return data.astype(np.int64)
        elif dtype == 'long':
            return data.astype(np.long)
        elif dtype == 'uint8':
            return data.astype(np.uint8)
        else:
            print(f'Type {dtype} not supported.')

    def convert(self, data, device=None, dtype='float32', indexed=None):
        return self.as_type(np.asarray(data), dtype)

    @staticmethod
    def pad(data, pad_width, mode='constant', value=None):
        if mode == 'constant' and value is not None:
            return np.pad(data, pad_width, mode, constant_values=value)
        if mode == 'linear_ramp' and value is not None:
            return np.pad(data, pad_width, mode, end_values=value)
        return np.pad(data, pad_width, mode)

    @staticmethod
    def matmul(matrix_a, matrix_b):
        return np.matmul(matrix_a, matrix_b)

    @staticmethod
    def transpose_2d(data):
        return np.transpose(data)

    @staticmethod
    def expand_dims(data, axis):
        return np.expand_dims(data, axis)

    @staticmethod
    def inverse(data):
        return np.linalg.inv(data)

    def create_tensor_from_list(self, data, dtype):
        return self.as_type(np.array(data), dtype)

    @staticmethod
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

    @staticmethod
    def cam2world(cam_pointcloud: np.ndarray, extrinsic_matrix: np.ndarray):
        view_matrix_ros_inv = np.linalg.inv(extrinsic_matrix)
        points_in_camera_axes_homogenous = np.pad(cam_pointcloud, ((0, 1), (0, 0)), constant_values=1.0)
        points_in_world_frame_homogenous = np.dot(view_matrix_ros_inv, points_in_camera_axes_homogenous)
        points_in_world_frame = points_in_world_frame_homogenous[:3, :].T

        return points_in_world_frame

    @staticmethod
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

    @staticmethod
    def get_extrinsic_matrix(view_transform: np.ndarray):
        view_transform = np.linalg.inv(view_transform)
        extrinsic_matrix = np.dot(
            np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]), np.linalg.inv(view_transform.T)
        )
        return extrinsic_matrix

    def get_view_matrix_ros(self, camera_params):
        """3D points in World Frame -> 3D points in Camera Ros Frame

        Returns:
            np.ndarray: the view matrix that transforms 3d points in the world frame to 3d points in the camera axes
                        with ros camera convention.
        """
        R_U_TRANSFORM = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        try:
            world_w_cam_u_T = self.transpose_2d(
                self.convert(
                    np.linalg.inv(camera_params['cameraViewTransform'].reshape(4, 4)),
                    dtype='float32',
                    indexed=True,
                )
            )
        except np.linalg.LinAlgError:
            world_w_cam_u_T = self.transpose_2d(
                self.convert(
                    UsdGeom.Imageable(self.camera_prim_path).ComputeLocalToWorldTransform(Usd.TimeCode.Default()),
                    dtype='float32',
                    indexed=True,
                )
            )
        r_u_transform_converted = self.convert(R_U_TRANSFORM, dtype='float32', indexed=True)
        return self.matmul(r_u_transform_converted, self.inverse(world_w_cam_u_T))

    def get_world_points_from_image_coords(
        self, depth: np.ndarray, mask: np.ndarray, extrinsic_matrix, intrinsic_matrix: np.ndarray
    ):
        """Using pinhole perspective projection, this method does the inverse projection given the depth of the
            pixels

        Args:
            extrinsic_matrix ():
            mask ():
            intrinsic_matrix ():
            depth (np.ndarray): depth corresponds to each of the pixel coords. shape is (n,)

        Returns:
            np.ndarray: (n, 3) 3d points (X, Y, Z) in world frame. shape is (n, 3) where n is the number of points.
        """
        points_in_camera_axes = self.get_camera_pointcloud(depth, mask, intrinsic_matrix)
        points_in_world_frame = self.cam2world(points_in_camera_axes, extrinsic_matrix)
        return points_in_world_frame

    def get_pointcloud(self, depth, camera_params) -> np.ndarray:
        """
        Returns:
            pointcloud (np.ndarray):  (N x 3) 3d points (X, Y, Z) in camera frame. Shape is (N x 3) where N is the number of points.
        Note:
            This currently uses the depth annotator to generate the pointcloud. In the future, this will be switched to use
            the pointcloud annotator.
        """
        camera_in = self.get_intrinsic_matrix(camera_params)
        camera_transform = camera_params['cameraViewTransform'].reshape(4, 4)
        camera_ex = self.get_extrinsic_matrix(camera_transform)
        # mask = (depth < max_depth) * (depth > min_depth)
        mask = (depth < 10000) * (depth > 0.01)
        pointcloud = self.get_world_points_from_image_coords(depth, mask, camera_ex, camera_in)

        return pointcloud

    # ====================================================================================

    def get_data(self) -> Dict:
        if self.config.enable:
            return self.get_camera_data(['landmarks', 'rgba', 'depth', 'camera_params', 'pointcloud'])
        return {}

    def clean(self) -> None:
        if self.rp is not None:
            log.debug('================ destroy render product =============')
            self.rp.destroy()
            self.rp = None

    def reset(self):
        log.debug('reset camera')
        self.clean()
        del self.camera_prim_path
        self.camera_prim_path = self.create_camera()
        self.init()

    def _get_face_to_instances(self, bbox: np.array, idToLabels):
        bbox = self._merge_tuples(bbox)
        label_to_bbox_area = []
        for row_idx in range(len(bbox)):
            id = str(bbox[row_idx][0])
            semantic_label = idToLabels[id]['class']
            bbox_area = bbox[row_idx][1]
            # occlusion = bbox[row_idx][2]
            # if bbox_area >= 0.02 * self.size[0] * self.size[1] or occlusion > 0.7:
            label_to_bbox_area.append((semantic_label, bbox_area))
        if not label_to_bbox_area:
            return []
        return [object_in_view[0] for object_in_view in label_to_bbox_area]

    def _merge_tuples(self, data):
        """
        Merge tuples with the same semanticId and compute weighted average for occlusionRatio
        based on the area of the bounding boxes.

        Parameters:
        data (list of tuples): Each tuple contains (semanticId, x_min, y_min, x_max, y_max, occlusionRatio)

        Returns:
        list of tuples: Merged tuples with (semanticId, total_area, weighted_average_occlusion_ratio)
        """
        # Dictionary to store the merged data
        merged_data = defaultdict(lambda: [0.0, 0.0])  # Initialize with total area and weighted occlusion sum

        # Traverse the original data and merge
        for entry in data:
            semantic_id, x_min, y_min, x_max, y_max, occlusion_ratio = entry
            area = (x_max - x_min) * (y_max - y_min)
            merged_data[semantic_id][0] += area  # Accumulate area
            merged_data[semantic_id][1] += occlusion_ratio * area  # Accumulate weighted occlusion_ratio

        # Construct the merged list
        result = []
        for semantic_id, values in merged_data.items():
            total_area, weighted_occlusion_sum = values
            weighted_average_occlusion_ratio = weighted_occlusion_sum / total_area if total_area != 0 else 0
            result.append((semantic_id, total_area, weighted_average_occlusion_ratio))

        return result
