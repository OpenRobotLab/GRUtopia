import json
import os

import numpy as np
import open3d as o3d
from pxr import Usd, UsdGeom
from shapely.geometry import MultiPoint, Point, Polygon

from grutopia.core.config.metric import MetricUserConfig
from grutopia.core.runtime.task_runtime import TaskRuntime
from grutopia.core.task.metric import BaseMetric
from grutopia.core.util import log

XY_DISTANCE_CLOSE_THRESHHOLD = 1.0


@BaseMetric.register('MobileManipulationSuccessMetric')
class MobileManipulationSuccessMetric(BaseMetric):
    """
    Calculate the success of this episode
    """

    def __init__(self, config: MetricUserConfig, task_runtime: TaskRuntime):
        super().__init__(config, task_runtime)
        self.extra = self.task_runtime.extra
        self.reset()

    def reset(self):
        self.position = None
        self.distance = 0.
        self.path = []
        # Setup stage
        self.root_path = '/Root'
        self.stage = Usd.Stage.Open(self.task_runtime.scene_asset_path)

        self.meshes_path = os.path.join(self.root_path, 'Meshes')
        self.meshes = self.stage.GetPrimAtPath(self.meshes_path)

        # Read meta data
        self.occupancy_map = np.load(os.path.join(self.extra['meta_path'], 'occupancy.npy'))
        with open(os.path.join(self.extra['meta_path'], 'annotation.json'), 'r') as fp:
            room_annotations = json.load(fp)
        with open(os.path.join(self.extra['meta_path'], 'object_dict.json'), 'r') as fp:
            self.object_dict = json.load(fp)

        # Process meta data
        self.room_dict = dict()
        for i, room_i_annotation in enumerate(room_annotations):
            room_i_type = room_i_annotation['room_type']
            # room_i_polygon = room_i_annotation["polygon"]
            room_i_key = f'{i}/{room_i_type}'
            self.room_dict[room_i_key] = room_i_annotation

    def update(self, task_obs: dict):
        """
        This function is called at each world step.
        """
        robot_obs = task_obs[self.task_runtime.robots[0].name]
        if robot_obs['render']:
            self.path.append(robot_obs['position'].tolist())
        if self.position is None:
            self.position = robot_obs['position'][:2]
            return
        self.distance += np.linalg.norm(self.position[:2] - robot_obs['position'][:2])
        self.position = robot_obs['position'][:2]

    def calc(self, task_info: dict):
        """
        This function is called to calculate the metrics when the episode is terminated.
        """
        log.info('MobileManipulationSuccessMetric calc() called.')
        success_list = []
        fail_list = []
        goal_conditions = task_info['conditions']
        instance_id_a = task_info['target']
        for condition_id, condition_dict in enumerate(goal_conditions):
            object_attribute = condition_dict['object_attribute']
            relationship = condition_dict['sample_relation']
            condition_type = 'space'
            instance_id_b = object_attribute['instance_id']
            # Space condition
            if condition_type == 'space':
                prim_a = self.get_prim(instance_id_a)
                point_cloud_info_a = get_point_cloud_info(prim_a)
                prim_b = self.get_prim(instance_id_b)
                point_cloud_info_b = get_point_cloud_info(prim_b)
                a2b, b2a = self.get_relationship_ab(point_cloud_info_a, point_cloud_info_b)
                if a2b == relationship:
                    success_list.append(condition_id)
                else:
                    fail_list.append(condition_id)
            else:
                raise NotImplementedError('Other evaluators not implemented!')
        success = len(fail_list) == 0
        # make sure all the result is in Python type
        return {
            'path': self.path,
            'pl': float(self.distance),
            'success': bool(success),
            'success_list': success_list,
            'fail_list': fail_list
        }

    def get_object_info(self, instance_id):
        object_info = self.object_dict[instance_id]
        return object_info

    def get_prim(self, instance_id):
        object_info = self.get_object_info(instance_id)
        path = os.path.join(self.meshes_path, object_info['scope'], instance_id)
        prim = self.stage.GetPrimAtPath(path)
        return prim

    def get_relationship_ab(self, point_cloud_info_a, point_cloud_info_b):
        spatial_relationship_a2b, spatial_relationship_b2a, dist = get_spatial_relationship(
            point_cloud_info_a, point_cloud_info_b)
        return spatial_relationship_a2b, spatial_relationship_b2a

    def get_room_pc(self, point_cloud):
        room_info = get_room_info(self.room_dict, self.occupancy_map)
        return get_room(point_cloud, room_info)


def get_room_info(room_dict, occupancy_map):

    def pixel_to_cord(map, p):
        i, j = p
        y = float(map[i, 0])
        x = float(map[0, j])
        return x, y

    room_infos = []
    for i, key in enumerate(room_dict):
        annotation = room_dict[key]
        v = annotation['polygon']
        k = f"{i}/{annotation['room_type']}"
        points = []
        for p in v:
            x, y = pixel_to_cord(occupancy_map, p)
            points.append((x, y))

        kernel = {
            'room_name': k,
            'shape': points,
        }
        room_infos.append(kernel)
    return room_infos


def get_room(point_cloud, room_info):
    total_point = len(point_cloud)
    rate = []
    dis_list = []
    for room in room_info:
        shape = room['shape']
        shape = Polygon(shape)

        pcd = point_cloud[..., :2]
        mp = MultiPoint(pcd)
        tmp = shape.intersection(mp)
        if type(tmp) == Point:  # if the intersection is empty
            cnt = 0
        else:
            cnt = len(tmp.geoms)

        r = 1.0 * cnt / total_point
        rate.append(r)
        dis_list.append(shape.distance(mp.convex_hull))

    room_index = np.argmax(rate)  # find the room with maximum overlap ratio
    if rate[room_index] < 0.1:  # if the maximum ratio is less than 0.1, find the closest room.
        room_index = np.argmin(dis_list)

    return room_info[room_index]['room_name']


def get_point_cloud_info(prim):
    mesh = get_mesh_via_prim(prim)
    point_cloud = mesh.sample_points_uniformly(number_of_points=7000)
    point_cloud = np.asarray(point_cloud.points) / 100
    min_points = point_cloud.min(0)
    max_points = point_cloud.max(0)
    point_cloud_info = [point_cloud, min_points, max_points]
    return point_cloud_info


def get_spatial_relationship(point_cloud_info_a, point_cloud_info_b):
    point_cloud_a, min_points_a, max_points_a = point_cloud_info_a
    point_cloud_b, min_points_b, max_points_b = point_cloud_info_b

    return infer_spatial_relationship(point_cloud_a, point_cloud_b, min_points_a, max_points_a, min_points_b,
                                      max_points_b)


def infer_spatial_relationship(point_cloud_a, point_cloud_b, min_points_a, max_points_a, min_points_b, max_points_b):
    xy_dist = calculate_xy_distance_between_two_point_clouds(point_cloud_a, point_cloud_b)
    # print(xy_dist)
    if xy_dist > XY_DISTANCE_CLOSE_THRESHHOLD:
        return None, None, None


def calculate_xy_distance_between_two_point_clouds(point_cloud_a: np.ndarray, point_cloud_b: np.ndarray):
    a = point_cloud_a[:, :2]
    b = point_cloud_b[:, :2]
    a = np.array(a)
    b = np.array(b)
    h1, c = a.shape
    h2, c = b.shape
    at = a.transpose()  # c x h
    bt = b.transpose()  # c x h
    at = at.reshape(c, h1, 1)
    bt = bt.reshape(c, 1, h2)
    at = at.repeat(h2, 2)
    bt = bt.repeat(h1, 1)
    sub = np.sqrt(((at - bt)**2).sum(0))
    return sub.min()


def get_mesh_via_prim(prim):
    points_total, faceuv_total, normals_total, faceVertexCounts_total, faceVertexIndices_total, mesh_total = recursive_parse(
        prim)
    faces = []
    count = 0
    for n in faceVertexCounts_total:
        f = [_ for _ in faceVertexIndices_total[count:count + n]]
        faces.append(f)
        count += n
    faces = np.array(faces)
    o3d_mesh = o3d.geometry.TriangleMesh()
    vertices = o3d.utility.Vector3dVector(points_total)
    triangles = o3d.utility.Vector3iVector(faces)
    o3d_mesh.vertices = vertices
    o3d_mesh.triangles = triangles

    return o3d_mesh


def recursive_parse(prim):

    def to_list(data):
        res = []
        if data is not None:
            res = [_ for _ in data]
        return res

    # print(prim.GetPath())
    translation = prim.GetAttribute('xformOp:translate').Get()
    if translation is None:
        translation = np.zeros(3)
    else:
        translation = np.array(translation)

    scale = prim.GetAttribute('xformOp:scale').Get()
    if scale is None:
        scale = np.ones(3)
    else:
        scale = np.array(scale)

    orient = prim.GetAttribute('xformOp:orient').Get()
    if orient is None:
        orient = np.zeros([4, 1])
        orient[0] = 1.0
    else:
        # print(orient)
        r = orient.GetReal()
        i, j, k = orient.GetImaginary()

        orient = np.array([r, i, j, k]).reshape(4, 1)

    transform = prim.GetAttribute('xformOp:transform').Get()
    if transform is None:
        transform = np.eye(4)
    else:
        transform = np.array(transform)

    rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(orient)

    points_total = []
    faceuv_total = []
    normals_total = []
    faceVertexCounts_total = []
    faceVertexIndices_total = []
    mesh_total = []
    if prim.IsA(UsdGeom.Mesh):
        mesh_path = str(prim.GetPath()).split('/')[-1]
        if not mesh_path == 'SM_Dummy':
            mesh_total.append(mesh_path)
            points = prim.GetAttribute('points').Get()
            normals = prim.GetAttribute('normals').Get()
            faceVertexCounts = prim.GetAttribute('faceVertexCounts').Get()
            faceVertexIndices = prim.GetAttribute('faceVertexIndices').Get()
            faceuv = prim.GetAttribute('primvars:st').Get()
            normals = to_list(normals)
            faceVertexCounts = to_list(faceVertexCounts)
            faceVertexIndices = to_list(faceVertexIndices)
            faceuv = to_list(faceuv)
            points = to_list(points)
            ps = []
            for p in points:
                x, y, z = p
                p = np.array((x, y, z))
                ps.append(p)

            points = ps

            base_num = len(points_total)
            for idx in faceVertexIndices:
                faceVertexIndices_total.append(base_num + idx)

            faceVertexCounts_total += faceVertexCounts
            faceuv_total += faceuv
            normals_total += normals
            points_total += points

    # else:

    children = prim.GetChildren()

    for child in children:
        points, faceuv, normals, faceVertexCounts, faceVertexIndices, mesh_list = recursive_parse(child)
        # child_path = child.GetPath()
        # if len(normals) > len(points):
        #     print(f"points is less than their normals, the prim is {child_path}")
        #     print(len(points), len(normals), len(points_total), len(normals_total), len(faceVertexCounts))

        base_num = len(points_total)
        for idx in faceVertexIndices:
            faceVertexIndices_total.append(base_num + idx)

        faceVertexCounts_total += faceVertexCounts
        faceuv_total += faceuv
        normals_total += normals[:len(points)]
        points_total += points
        mesh_total += mesh_list

    new_points = []
    for i, p in enumerate(points_total):
        pn = np.array(p)
        pn *= scale
        pn = np.matmul(rotation_matrix, pn)
        pn += translation
        new_points.append(pn)

    if len(new_points) > 0:
        points_mat = np.ones((len(new_points), 4)).astype(np.float32)
        points_mat[:, :3] = np.array(new_points)
        # print(points_mat.shape, transform.shape)
        points_mat = np.matmul(points_mat, transform)
        new_points = [_ for _ in points_mat[:, :3]]

    return new_points, faceuv_total, normals_total, faceVertexCounts_total, faceVertexIndices_total, mesh_total
