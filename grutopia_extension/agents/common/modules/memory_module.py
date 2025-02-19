import os
from typing import List, Tuple, Union

import cv2
import matplotlib.pyplot as plt
import numpy as np
import torch
from agent_utils.acyclic_enforcer import AcyclicEnforcer
from agent_utils.geometry_utils import extract_camera_pos_zyxrot, get_intrinsic_matrix
from modules.mapping.object_point_cloud_map import ObjectPointCloudMap
from modules.mapping.obstacle_map import ObstacleMap
from modules.mapping.value_map import ValueMap
from modules.vlm.blip2 import BLIP2Client
from modules.vlm.blip2itm import BLIP2ITMClient
from modules.vlm.coco_classes import COCO_CLASSES
from modules.vlm.grounding_dino import GroundingDINOClient, ObjectDetections
from modules.vlm.large_model import Vlm_gpt4o
from modules.vlm.sam import MobileSAMClient
from modules.vlm.yolov7 import YOLOv7Client
from transformers import CLIPModel, CLIPProcessor

PROMPT_SEPARATOR = '|'
GRU_ID_TO_NAME = [
    'counter',
    'plate',
    'bed',
    'pan',
    'dishwasher',
    'mirror',
    'telephone',
    'keyboard',
    'nightstand',
    'tvstand',
    'basket',
    'bowl',
    'box',
    'laptop',
    'clock',
    'sink',
    'mouse',
    'electric cooker',
    'cabinet',
    'fan',
    'sofa chair',
    'plant',
    'backpack',
    'trashcan',
    'shelf',
    'sideboard cabinet',
    'toilet',
    'washing machine',
    'couch',
    'pot',
    'refrigerator',
    'lamp',
    'chest of drawers',
    'bottle',
    'oven',
    'toy',
    'pillow',
    'bathtub',
    'tray',
    'desk',
    'pen',
    'microwave',
    'faucet',
    'cart',
    'shoe',
    'towel',
    'shoe cabinet',
    'picture',
    'ceiling light',
    'tv',
    'cup',
    'curtain',
    'decoration',
    'hearth',
    'table',
    'piano',
    'light',
    'chair',
    'clothes',
    'monitor',
    'blanket',
    'tea table',
    'stool',
    'coffee maker',
]


class Memory:
    def __init__(self, llm: Vlm_gpt4o, memory_config: dict):
        self.task_config = memory_config['task_config']
        self.map_config = memory_config['map_config']
        self.obstacle_map_config = memory_config['obstacle_map']
        self.value_map_config = memory_config['value_map']
        self.object_map_config = memory_config['object_map']

        self.processor = CLIPProcessor.from_pretrained('openai/clip-vit-base-patch32')
        self.model = CLIPModel.from_pretrained('openai/clip-vit-base-patch32')

        if self.object_map_config['use_vqa']:
            self._vqa = BLIP2Client(port=int(os.environ.get('BLIP2_PORT', '12185')))

        self._object_detector = GroundingDINOClient(port=int(os.environ.get('GROUNDING_DINO_PORT', '12181')))

        self._coco_object_detector = YOLOv7Client(port=int(os.environ.get('YOLOV7_PORT', '12184')))
        self._mobile_sam = MobileSAMClient(port=int(os.environ.get('SAM_PORT', '12183')))
        self._itm = BLIP2ITMClient(port=int(os.environ.get('BLIP2ITM_PORT', '12182')))

        self._non_coco_caption = ' . '.join(GRU_ID_TO_NAME).replace('|', ' . ') + ' .'
        self.past_frontier = np.array([0, 0])
        self.llm = llm

    def reset(self, goal):
        self.past_frontier = np.array([0, 0])
        self._last_frontier = np.zeros(2)
        self.obstacle_map = ObstacleMap(**self.obstacle_map_config)
        self.value_map = ValueMap(
            size=self.value_map_config['size'],
            value_channels=self.value_map_config['value_channels'],
            pixels_per_meter=self.map_config['pixels_per_meter'],
        )
        self.object_map = ObjectPointCloudMap(erosion_size=self.object_map_config['erosion_size'])
        self._acyclic_enforcer = AcyclicEnforcer()

        self.goal = goal
        self.goal_infos = {'room': [], 'spatial': [], 'attribute': []}

    def reset_goal(self, goal):
        self.value_map.reset()
        self.object_map.reset()

        self.goal = goal
        self.goal_infos = {'room': [], 'spatial': [], 'attribute': []}

    def update(self, observations):
        memory_update = False

        # Update map
        try:
            depth = observations['sensors']['camera']['depth']
        except KeyError:
            depth = None
        if depth is not None and len(depth) > 0:
            memory_update = True
            camera_transform = observations['sensors']['camera']['camera_params']['cameraViewTransform'].reshape(4, 4)
            camera_in = get_intrinsic_matrix(observations['sensors']['camera']['camera_params'])
            topdown_fov = 2 * np.arctan(camera_in[0, 2] / camera_in[0, 0])
            camera_position, camera_rotation = extract_camera_pos_zyxrot(camera_transform)

            rgb = observations['sensors']['camera']['rgba']

            # Update obstacle map
            self.obstacle_map.update_map(
                depth,
                camera_in,
                camera_transform,
                self.map_config['min_depth'],
                self.map_config['max_depth'],
                topdown_fov,
                verbose=self.task_config['verbose'],
                agent_path=self.task_config['agent_path'],
            )

            self.obstacle_map.update_agent_traj(camera_position[:2], camera_rotation[0])

            # Update value map
            cosine = [
                self._itm.cosine(
                    rgb,
                    p.replace('target_object', self.goal.replace('|', '/')),
                )
                for p in self.value_map_config['text_prompt'].split(PROMPT_SEPARATOR)
            ]

            self.value_map.update_map(
                np.array(cosine),
                depth,
                camera_transform,
                self.map_config['min_depth'],
                self.map_config['max_depth'],
                topdown_fov,
            )
            self.value_map.update_agent_traj(camera_position[:2], camera_rotation[0])
            if self.task_config['verbose']:
                plt.imsave(
                    os.path.join(self.task_config['agent_path'], 'images/tp_rgb.jpg'),
                    observations['sensors']['tp_camera']['rgba'],
                )
                plt.imsave(os.path.join(self.task_config['agent_path'], 'images/rgb.jpg'), rgb)
                plt.imsave(
                    os.path.join(self.task_config['agent_path'], 'images/depth.jpg'),
                    depth,
                )
                plt.imsave(
                    os.path.join(self.task_config['agent_path'], 'images/value_map.jpg'),
                    self.value_map._map,
                )

            # Update object map
            self._update_object_map(
                rgb,
                depth,
                camera_transform,
                camera_in,
                self.map_config['min_depth'],
                self.map_config['max_depth'],
                topdown_fov,
            )
        return memory_update

    def get_best_frontier(
        self,
        observations: dict,
    ) -> Tuple[np.ndarray, float]:
        """Returns the best frontier and its value based on self._value_map.

        Args:
            observations (Union[Dict[str, Tensor], "TensorDict"]): The observations from
                the environment.
            frontiers (np.ndarray): The frontiers to choose from, array of 2D points.

        Returns:
            Tuple[np.ndarray, float]: The best frontier and its value.
        """
        # The points and values will be sorted in descending order
        sorted_pts, sorted_values = self._sort_frontiers_by_value(self.obstacle_map.frontiers)
        robot_xy = observations['position'][:2]
        best_frontier_idx = None
        top_two_values = tuple(sorted_values[:2])

        os.environ['DEBUG_INFO'] = ''
        # # If there is a last point pursued, then we consider sticking to pursuing it
        # # if it is still in the list of frontiers and its current value is not much
        # # worse than self._last_value.

        # if not np.array_equal(self._last_frontier, np.zeros(2)):
        #     curr_index = None

        #     for idx, p in enumerate(sorted_pts):
        #         if np.array_equal(p, self._last_frontier):
        #             # Last point is still in the list of frontiers
        #             curr_index = idx
        #             break

        #     if curr_index is None:
        #         closest_index = closest_point_within_threshold(sorted_pts, self._last_frontier, threshold=0.5)

        #         if closest_index != -1:
        #             # There is a point close to the last point pursued
        #             curr_index = closest_index

        #     if curr_index is not None:
        #         curr_value = sorted_values[curr_index]
        #         if curr_value + 0.01 > self._last_value:
        #             # The last point pursued is still in the list of frontiers and its
        #             # value is not much worse than self._last_value
        #             print("Sticking to last point.")
        #             os.environ["DEBUG_INFO"] += "Sticking to last point. "
        #             best_frontier_idx = curr_index

        # If there is no last point pursued, then just take the best point, given that
        # it is not cyclic.
        if best_frontier_idx is None:
            for idx, frontier in enumerate(sorted_pts):
                cyclic = self._acyclic_enforcer.check_cyclic(robot_xy, frontier, top_two_values)
                if cyclic:
                    print('Suppressed cyclic frontier.')
                    continue
                if np.linalg.norm(frontier - self.past_frontier) < 0.01:
                    print('Suppressed repeated frontier')
                    continue

                best_frontier_idx = idx
                self.past_frontier = frontier
                break

        if best_frontier_idx is None:
            print('All frontiers are cyclic. Just choosing the closest one.')
            os.environ['DEBUG_INFO'] += 'All frontiers are cyclic. '
            best_frontier_idx = max(
                range(len(self.obstacle_map.frontiers)),
                key=lambda i: np.linalg.norm(self.obstacle_map.frontiers[i] - robot_xy),
            )

        best_frontier = sorted_pts[best_frontier_idx]
        best_value = sorted_values[best_frontier_idx]
        self._acyclic_enforcer.add_state_action(robot_xy, best_frontier, top_two_values)
        self._last_value = best_value
        self._last_frontier = best_frontier
        os.environ['DEBUG_INFO'] += f' Best value: {best_value*100:.2f}%'

        return best_frontier, best_value

    def _sort_frontiers_by_value(self, frontiers: np.ndarray) -> Tuple[np.ndarray, List[float]]:
        sorted_frontiers, sorted_values = self.value_map.sort_waypoints(frontiers, 0.5)
        return sorted_frontiers, sorted_values

    def update_goal_info(self, dialogue: dict):
        new_goal_infos = self.llm.get_answer('extract_info', question=dialogue['question'], answer=dialogue['answer'])
        if new_goal_infos is not None:
            new_goal_infos = new_goal_infos.strip().split('\n')
            new_goal_infos = [new_goal_info.split(',') for new_goal_info in new_goal_infos]
            for new_goal_info in new_goal_infos:
                if len(new_goal_info) != 2:
                    continue
                if 'relation' in new_goal_info[0].lower():
                    self.goal_infos['spatial'].append(new_goal_info[1].strip())
                elif 'room' in new_goal_info[0].lower():
                    self.goal_infos['room'].append(new_goal_info[1].strip())
                elif 'attribute' in new_goal_info[0].lower():
                    self.goal_infos['attribute'].append(new_goal_info[1].strip())

    def _update_object_map(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        tf_camera_to_episodic: np.ndarray,
        camera_in: np.ndarray,
        min_depth: float,
        max_depth: float,
        cone_fov: float,
    ):
        """
        Updates the object map with the given rgb and depth images, and the given
        transformation matrix from the camera to the episodic coordinate frame.

        Args:
            rgb (np.ndarray): The rgb image to use for updating the object map. Used for
                object detection and Mobile SAM segmentation to extract better object
                point clouds.
            depth (np.ndarray): The depth image to use for updating the object map. It
                is normalized to the range [0, 1] and has a shape of (height, width).
            tf_camera_to_episodic (np.ndarray): The transformation matrix from the
                camera to the episodic coordinate frame.
            min_depth (float): The minimum depth value (in meters) of the depth image.
            max_depth (float): The maximum depth value (in meters) of the depth image.
            fx (float): The focal length of the camera in the x direction.
            fy (float): The focal length of the camera in the y direction.

        Returns:
            ObjectDetections: The object detections from the object detector.
        """
        detections = self._get_object_detections(rgb)
        height, width = rgb.shape[:2]
        self._object_masks = np.zeros((height, width), dtype=np.uint8)
        for idx in range(len(detections.logits)):
            bbox_denorm = detections.boxes[idx] * np.array([width, height, width, height])
            object_mask = self._mobile_sam.segment_bbox(rgb, bbox_denorm.tolist())
            # If we are using vqa, then use the BLIP2 model to visually confirm whether
            # the contours are actually correct.
            contours, _ = cv2.findContours(object_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            annotated_rgb = cv2.drawContours(rgb.copy(), contours, -1, (255, 0, 0), 2)
            if self.object_map_config['use_vqa']:
                question = f"Question: {self.object_map_config['vqa_prompt']}"
                if not detections.phrases[idx].endswith('ing'):
                    question += 'a '
                question += detections.phrases[idx] + '? Answer:'
                answer = self._vqa.ask(annotated_rgb, question)
                if not answer.lower().startswith('yes'):
                    continue
            if self.task_config['verbose']:
                plt.imsave(
                    os.path.join(self.task_config['agent_path'], 'images/annotated.jpg'),
                    annotated_rgb,
                )

            self._object_masks[object_mask > 0] = 1
            self.object_map.update_map(
                rgb,
                depth,
                object_mask,
                tf_camera_to_episodic,
                camera_in,
                min_depth,
                max_depth,
            )
            if self.task_config['verbose'] and len(self.object_map.clouds) > 0:
                np.save(
                    os.path.join(self.task_config['agent_path'], 'images/pointclouds.npy'),
                    self.object_map.clouds[0]['clouds'][:, :3],
                )
        # self.object_map.update_explored(tf_camera_to_episodic, max_depth, cone_fov)

        return detections

    def _get_object_detections(self, img: np.ndarray) -> ObjectDetections:
        dino_inputs = self.processor(text=GRU_ID_TO_NAME + [self.goal], return_tensors='pt', padding=True)
        yolo_inputs = self.processor(text=COCO_CLASSES + [self.goal], return_tensors='pt', padding=True)
        with torch.no_grad():
            dino_outputs = self.model.get_text_features(**dino_inputs)
            yolo_outputs = self.model.get_text_features(**yolo_inputs)
        dino_class = GRU_ID_TO_NAME[
            torch.nn.functional.cosine_similarity(dino_outputs[-1].unsqueeze(0), dino_outputs[:-1]).argmax()
        ]
        yolo_class = COCO_CLASSES[
            torch.nn.functional.cosine_similarity(yolo_outputs[-1].unsqueeze(0), yolo_outputs[:-1]).argmax()
        ]
        target_classes = [dino_class, yolo_class]

        has_coco = any(c in COCO_CLASSES for c in target_classes)

        detections = (
            self._coco_object_detector.predict(img)
            if has_coco
            else self._object_detector.predict(img, caption=self._non_coco_caption)
        )
        detections.filter_by_class(target_classes)
        det_conf_threshold = (
            self.object_map_config['coco_threshold'] if has_coco else self.object_map_config['non_coco_threshold']
        )
        detections.filter_by_conf(det_conf_threshold)

        if has_coco and detections.num_detections == 0:
            # Retry with non-coco object detector
            detections = self._object_detector.predict(img, caption=self._non_coco_caption)
            detections.filter_by_class(target_classes)
            detections.filter_by_conf(self.object_map_config['non_coco_threshold'])

        return detections

    def _get_target_object_location(self, position: np.ndarray) -> Union[None, np.ndarray]:
        if self.object_map.has_object(self.goal):
            return self.object_map.get_best_object(self.goal, position)
        else:
            return None
