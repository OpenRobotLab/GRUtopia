from collections import defaultdict
from typing import Dict

import numpy as np
import omni.replicator.core as rep

from grutopia.core.robot.robot import BaseRobot, Scene
from grutopia.core.robot.robot_model import SensorModel
from grutopia.core.robot.sensor import BaseSensor
from grutopia.core.util import log


@BaseSensor.register('RepCamera')
class RepCamera(BaseSensor):
    """
    wrap of isaac sim's Camera class
    """

    def __init__(self, config: SensorModel, robot: BaseRobot, name: str = None, scene: Scene = None):
        super().__init__(config, robot, scene)
        self.name = name
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
        self.size = (512, 512)
        # Use the configured camera size if provided.
        if self.config.size is not None:
            self.size = self.config.size

        prim_path = self._robot.user_config.prim_path + '/' + self.config.prim_path
        log.debug('camera_prim_path: ' + prim_path)
        log.debug('name            : ' + self.config.name)
        log.debug(f'size            : {self.size}')
        return prim_path

    def get_camera_data(self, data_names):
        """
        Get specified data from a camera.

        Parameters:
            camera: str or rep.Camera, the prim_path of the camera or a camera object created by rep.create.camera
            resolution: tuple, the resolution of the camera, e.g., (1920, 1080)
            data_names: list, a list of desired data names, can be any combination of "landmarks", "rgba", "depth", "pointcloud", "camera_params"

        Returns:
            output_data: dict, a dict of data corresponding to the requested data names
        """

        output_data = {}

        # # Create a render product for the specified camera and resolution
        # rp = rep.create.render_product(self.camera_prim_path, self.size)

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
            self.rp_annotators['distance_to_image_plane'] = distance_to_image_plane

        if 'pointcloud' in data_names and 'pointcloud' not in self.rp_annotators:
            pointcloud = rep.AnnotatorRegistry.get_annotator('pointcloud')
            pointcloud.attach(self.rp)
            self.rp_annotators['pointcloud'] = pointcloud

        if 'camera_params' in data_names and 'camera_params' not in self.rp_annotators:
            camera_params = rep.annotators.get('CameraParams').attach(self.rp)
            self.rp_annotators['camera_params'] = camera_params

        for name, annotator in self.rp_annotators.items():
            if name == 'landmarks':
                output_data['bounding_box_2d_tight'] = annotator.get_data()
                if len(output_data['bounding_box_2d_tight']['data']) > 0:
                    output_data['landmarks'] = self._get_face_to_instances(
                        output_data['bounding_box_2d_tight']['data'],
                        output_data['bounding_box_2d_tight']['info']['idToLabels'])
                else:
                    output_data['landmarks'] = []
                continue
            output_data[name] = annotator.get_data()

        return output_data

    def get_data(self) -> Dict:
        if self.config.enable:
            return self.get_camera_data(['landmarks', 'rgba', 'depth', 'pointcloud', 'camera_params'])
        return {}

    def cleanup(self) -> None:
        if self.rp is not None:
            log.debug('================ destroy render product =============')
            self.rp.destroy()
            self.rp = None

    def reset(self):
        log.debug('reset camera')
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
