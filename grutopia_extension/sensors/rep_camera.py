from collections import OrderedDict, defaultdict

import numpy as np
import omni.replicator.core as rep
import torch

from grutopia.core.robot.robot import BaseRobot, Scene
from grutopia.core.sensor.sensor import BaseSensor
from grutopia.core.util import log
from grutopia_extension.configs.sensors import RepCameraCfg


@BaseSensor.register('RepCamera')
class RepCamera(BaseSensor):
    """
    wrap of replicator render_product
    """

    def __init__(self, config: RepCameraCfg, robot: BaseRobot, scene: Scene = None):
        super().__init__(config, robot, scene)
        self.resolution = None
        self.config = config
        self.depth = config.depth
        self.rp = None
        self.rp_annotators = {}

    def post_reset(self):
        self.restore_sensor_info()

    def restore_sensor_info(self):
        self.cleanup()
        self.camera_prim_path = self.create_camera()
        if self.rp is None:
            self.rp = rep.create.render_product(self.camera_prim_path, self.resolution)

    def create_camera(self) -> str:
        """Create an isaac-sim camera object.

        Initializes the camera's resolution and prim path based on configuration.

        Returns:
            i_Camera: The initialized camera object.
        """
        # Initialize the default resolution for the camera.
        self.resolution = (64, 64)
        # Use the configured camera resolution if provided.
        if self.config.resolution is not None:
            self.resolution = self.config.resolution

        prim_path = self._robot.config.prim_path + '/' + self.config.prim_path
        log.debug('================ create camera ===============')
        log.debug('camera_prim_path: ' + prim_path)
        log.debug('name            : ' + self.config.name)
        log.debug(f'resolution      : {self.resolution}')
        return prim_path

    def get_camera_data(self, data_names=('landmarks', 'rgba', 'depth', 'pointcloud', 'camera_params')) -> OrderedDict:
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

        if self.depth and 'depth' in data_names and 'depth' not in self.rp_annotators:
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

        if self.depth:
            try:
                output_data['pointcloud'] = self.get_pointcloud_gpu(output_data['depth'], output_data['camera_params'])
            except Exception:
                output_data['pointcloud'] = None
        return self._make_ordered(output_data)

    # ============================================================================================================
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

    def get_pointcloud_gpu(self, depth, camera_params) -> np.ndarray:
        """
        Converts a depth image to a point cloud using GPU acceleration.

        Args:
            depth (np.ndarray): The depth image as a 2D numpy array.
            camera_params (dict): The camera parameters, including intrinsic and extrinsic matrices.

        Returns:
            np.ndarray: The point cloud as a 2D numpy array with shape (N, 3), where N is the number of points.
        """
        # Convert the depth image to a GPU tensor.
        depth_tensor = torch.as_tensor(depth, device='cuda', dtype=torch.float32)

        # Create a mask for valid depth values.
        mask = (depth_tensor < 10000) & (depth_tensor > 0.01)

        # Get the indices of valid depth values.
        v, u = torch.where(mask)
        # If no valid depth values, return an empty array.
        if v.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float32)

        # Extract depth values at valid indices.
        depth_values = depth_tensor[v, u]

        # Convert 2D indices to 2D points.
        points_2d = torch.stack([u, v], dim=1).float()  # (N, 2)
        # Convert 2D points to homogeneous coordinates.
        homogeneous = torch.cat([points_2d, torch.ones_like(points_2d[:, :1])], dim=1)  # (N, 3)

        # Get the inverse of the intrinsic matrix.
        intrinsic_matrix = self.get_intrinsic_matrix(camera_params)
        K_inv = torch.inverse(torch.tensor(intrinsic_matrix, device='cuda', dtype=torch.float32))
        # Get points in camera local frame.
        points_camera = (K_inv @ (homogeneous.T * depth_values).T[:, :, None]).squeeze()

        # Get the extrinsic matrix.
        camera_transform = torch.tensor(
            camera_params['cameraViewTransform'].reshape(4, 4), device='cuda', dtype=torch.float32
        )
        extrinsic_matrix = self.get_extrinsic_matrix_gpu(camera_transform)
        view_matrix_ros_inv = torch.inverse(extrinsic_matrix)

        # Convert points in camera frame to homogeneous coordinates.
        ones = torch.ones(points_camera.shape[0], 1, device='cuda')
        points_camera_homo = torch.cat([points_camera, ones], dim=1)  # (N, 4)
        # Translate points to world frame.
        points_world_homo = points_camera_homo @ view_matrix_ros_inv.T
        points_world = points_world_homo[:, :3].cpu().numpy()

        return points_world

    def get_extrinsic_matrix_gpu(self, view_transform: torch.Tensor) -> torch.Tensor:
        R_U_TRANSFORM = torch.tensor(
            [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]], device='cuda', dtype=torch.float32
        )

        return R_U_TRANSFORM @ view_transform.T

    # ====================================================================================

    def get_data(self) -> OrderedDict:
        if self.config.enable:
            return self.get_camera_data()
        return self._make_ordered()

    def cleanup(self) -> None:
        if self.rp is not None:
            for anno in self.rp_annotators.values():
                anno.detach(self.rp)
            del self.rp_annotators
            self.rp_annotators = {}
            self.rp = None

    def _get_face_to_instances(self, bbox: np.array, idToLabels):
        bbox = self._merge_tuples(bbox)
        label_to_bbox_area = []
        for row_idx in range(len(bbox)):
            id = str(bbox[row_idx][0])
            semantic_label = idToLabels[id]['class']
            bbox_area = bbox[row_idx][1]
            # occlusion = bbox[row_idx][2]
            # if bbox_area >= 0.02 * self.resolution[0] * self.resolution[1] or occlusion > 0.7:
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
