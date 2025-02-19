import os
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
from agent_utils.path_finder import (
    TimeLimitedBiAStarFinder,
    find_nearest_free_point,
    find_nearest_reset_point,
    is_line_collision_free,
    simplify_path_with_collision_check,
)
from deepdiff import DeepDiff
from modules.memory_module import Memory
from modules.planning_module import Planning
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from scipy.spatial.transform import Rotation as R


class Actuator:
    def __init__(self, actuator_config):
        self.map_config = actuator_config['map_config']
        self.task_config = actuator_config['task_config']

    def reset(self):
        self.initial_exploration_goals = None
        self.sub_goals = []
        self.sub_goal_idx = 0
        self.candidates_idx = 0
        self.nav_goal = None
        self.recover_action = None
        self.nav_path = []
        self.path = []

    def init_exploration(self, observation):
        finish = False
        sim_step = observation['sim_step']
        current_position = observation['position']
        current_orientation = observation['orientation']
        yaw = R.from_quat([*current_orientation[1:], current_orientation[0]]).as_euler('ZYX')[0]
        if self.initial_exploration_goals is None:
            self.initial_exploration_goals = [
                (yaw + degree) % (2 * np.pi) - (2 * np.pi)
                if (yaw + degree) % (2 * np.pi) > np.pi
                else (yaw + degree) % (2 * np.pi)
                for degree in np.linspace(2 * np.pi, 0, 10, endpoint=False)
            ]
            self.initial_exploration_goal = self.initial_exploration_goals.pop()

        if (
            len(self.initial_exploration_goals) == 0
            or (current_position[2] < self.task_config['fall_threshold'])
            or sim_step > 1000
        ):  # noqa
            finish = True
        elif abs(yaw - self.initial_exploration_goal) < 0.1:
            self.initial_exploration_goal = self.initial_exploration_goals.pop()
        quat = R.from_euler('z', self.initial_exploration_goal, degrees=False).as_quat()
        return finish, {'rotate': [[quat[-1], *quat[:3]]]}

    def navigate(self, observation: dict, memory: Memory, planning: Planning, render: bool):
        finish = 0
        current_position = observation['position']

        # If fall down or get stuck or have collision
        if render:
            self.path.append(
                {
                    'position': observation['position'],
                    'orientation': observation['orientation'],
                }
            )
            is_fall, reset_point = self._check_fall(observation, 'navigate', memory=memory)
            if is_fall and self.task_config['robot_type'] == 'h1':
                self.nav_goal = None
                self.recover_action = [np.array([*reset_point, 1.05])]
                return finish, {'recover': self.recover_action}

            if observation['controllers']['move_along_path']:
                next_target = (
                    self.nav_path[observation['controllers']['move_along_path']['current_index'] + 1]
                    if len(self.nav_path) > observation['controllers']['move_along_path']['current_index'] + 1
                    else observation['controllers']['move_along_path']['current_point']
                )
                from_point = tuple(
                    self._xy_to_px(observation['controllers']['move_along_path']['current_point'].squeeze()[:2])
                    .squeeze()
                    .astype(int)
                )
                to_point = tuple(self._xy_to_px(np.array(next_target).squeeze()[:2]).squeeze().astype(int))
                if not is_line_collision_free(memory.obstacle_map._navigable_map, from_point, to_point):
                    self.nav_goal = None

        if self.nav_goal is None:
            self.nav_goal = (
                self.sub_goals[self.sub_goal_idx] if self.sub_goal_idx < len(self.sub_goals) else planning.content
            )
            self.nav_path = self._navigate_p2p(current_position[:2], self.nav_goal['goal'], memory)
            return finish, {'move_along_path': [self.nav_path]}

        # If new candidate found while exploration
        if self.nav_goal['type'] == 'frontier' and len(memory.object_map.clouds) > self.candidates_idx:
            change_flag = False
            for cloud_idx, clouds_data in enumerate(memory.object_map.clouds.values()):
                if cloud_idx >= self.candidates_idx:
                    object_position = np.mean(clouds_data['clouds'], axis=0)[:2]
                    if (
                        np.linalg.norm(object_position - np.array(current_position[:2]))
                        > self.task_config['no_explore_threshold']
                    ):
                        change_flag = True
                        self.sub_goals.append({'type': 'frontier', 'goal': object_position})
            self.candidates_idx = len(memory.object_map.clouds)
            is_equal = not DeepDiff(self.nav_goal, planning.content, ignore_order=True)
            self.nav_goal = None if is_equal and change_flag else self.nav_goal
            return finish, {'move_along_path': [self.nav_path]}

        # If navigate finish: no path found or goal reached
        if render and (len(self.nav_path) == 0 or observation['controllers']['move_along_path']['finished']):
            is_equal = not DeepDiff(self.nav_goal, planning.content, ignore_order=True)
            finish = 1 if self.nav_goal['type'] == 'frontier' and is_equal else 2
            self.sub_goal_idx = (
                self.sub_goal_idx + 1 if self.nav_goal['type'] == 'frontier' and not is_equal else self.sub_goal_idx
            )
            self.nav_goal = None
            return finish, {}

        return finish, {'move_along_path': [self.nav_path]}

    def rotate(self, observation: dict, memory: Memory, nav_goal: list, render: bool):
        finish = False
        nav_goal = np.array(nav_goal).squeeze()[:2]
        current_position = observation['position']
        current_orientation = observation['orientation']
        yaw = R.from_quat([*current_orientation[1:], current_orientation[0]]).as_euler('ZYX')[0]
        if render:
            self.path.append(
                {
                    'position': observation['position'],
                    'orientation': observation['orientation'],
                }
            )
            is_fall, reset_values = self._check_fall(observation, 'rotate', memory, reset_p=nav_goal)
            if is_fall and self.task_config['robot_type'] == 'h1':
                self.recover_action = [
                    np.array([*reset_values[0], 1.05]),
                    reset_values[1],
                ]
                return finish, {'recover': self.recover_action}

        if abs(yaw - np.arctan2(nav_goal[1] - current_position[1], nav_goal[0] - current_position[0])) < 0.02:
            finish = True
            nav_goal = None
            return finish, {}
        else:
            quat = R.from_euler(
                'z',
                np.arctan2(nav_goal[1] - current_position[1], nav_goal[0] - current_position[0]),
                degrees=False,
            ).as_quat()
            return finish, {'rotate': [[quat[-1], *quat[:3]]]}

    def _check_fall(
        self,
        observation: dict,
        state: str,
        memory: Optional[Memory] = None,
        reset_p: np.ndarray = None,
    ):
        current_position = observation['position']
        current_px = self._xy_to_px(current_position.squeeze()[:2]).squeeze().astype(int)
        offset = np.linalg.norm(
            np.mean([path['position'][:2] for path in self.path][-5:], axis=0) - observation['position'][:2]
        ) + np.linalg.norm(
            np.mean([path['orientation'] for path in self.path][-5:], axis=0) - observation['orientation']
        )
        if (current_position[2] < self.task_config['fall_threshold']) or (len(self.path) >= 5 and offset < 0.05):
            if state == 'rotate':
                nearest_free_space = find_nearest_reset_point(
                    tuple(current_px),
                    memory.obstacle_map.seen_map * memory.obstacle_map._navigable_map,
                    self.map_config['agent_radius'] * self.map_config['pixels_per_meter'],
                )
                nearest_free_space = self._px_to_xy(nearest_free_space).squeeze()
                quat = R.from_euler(
                    'z',
                    np.arctan2(
                        reset_p[1] - current_position[1],
                        reset_p[0] - current_position[0],
                    ),
                    degrees=False,
                ).as_quat()
                reset_value = [nearest_free_space, np.array([quat[-1], *quat[:3]])]
            elif state == 'navigate':
                if observation['controllers']['move_along_path'].get('current_point') is not None:
                    nearest_free_space = observation['controllers']['move_along_path']['current_point'].squeeze()[:2]
                else:
                    nearest_free_space = find_nearest_reset_point(
                        tuple(current_px),
                        memory.obstacle_map.seen_map * memory.obstacle_map._navigable_map,
                        self.map_config['agent_radius'] * self.map_config['pixels_per_meter'],
                    )
                    nearest_free_space = self._px_to_xy(nearest_free_space).squeeze()
                reset_value = np.array(nearest_free_space)
            else:
                assert 'Illegal state meet!'
            if self.task_config['verbose']:
                reset_point = self._xy_to_px(nearest_free_space).squeeze()
                plt.figure(figsize=(10, 10))
                plt.imshow(memory.obstacle_map._navigable_map, cmap='gray')
                plt.scatter(
                    [reset_point[0], current_px[0]],
                    [reset_point[1], current_px[1]],
                    color='red',
                    marker='o',
                    s=10,
                )
                plt.title('Navigable Map')
                plt.savefig(os.path.join(self.task_config['agent_path'], 'images/reset_point.jpg'))
            return True, reset_value
        else:
            return False, None

    def _navigate_p2p(self, start: np.ndarray, goal: np.ndarray, memory: Memory) -> list:
        # 1. Get the navigable map
        # 0 represents obstacles, 1 represents navigable space
        navigable_map = memory.obstacle_map._navigable_map
        seen_map = memory.obstacle_map.seen_map
        # 2. Get the start and goal points
        start_point = self._xy_to_px(start.squeeze()[:2]).squeeze().astype(int)
        goal_point = self._xy_to_px(goal.squeeze()[:2]).squeeze().astype(int)

        # 3. Find the nearest free points to start and goal
        start = find_nearest_free_point(tuple(start_point), navigable_map * seen_map)
        if start is None:
            start = find_nearest_free_point(tuple(start_point), navigable_map)
        goal = find_nearest_free_point(tuple(goal_point), navigable_map * seen_map)
        if goal is None:
            goal = find_nearest_free_point(tuple(goal_point), navigable_map)

        if start == goal:
            return [[*self._px_to_xy(goal).squeeze(), 1.05]]

        if start is None or goal is None:
            print('Cannot find valid start or goal point.')
            return []

        # 4. Prepare the grid for pathfinding
        matrix = navigable_map.tolist()
        grid = Grid(matrix=matrix)

        # 5. Set up the start and end nodes
        grid_start = grid.node(start[0], start[1])
        grid_end = grid.node(goal[0], goal[1])

        # 6. Perform A* pathfinding
        finder = TimeLimitedBiAStarFinder(time_limit=10, diagonal_movement=DiagonalMovement.always)
        path = finder.find_path(grid_start, grid_end, grid)

        if self.task_config['verbose']:
            if path and path[-1] == grid_end:
                print('Path found to the goal.')
            else:
                print('No path to the goal found within the time limit.')
                print('Returning path to the closest point.')

            print(f'Original Path Length: {len(path)}')

        # 7. Simplify the path with collision checking
        path = [(point.x, point.y) for point in path]
        simplified_path = simplify_path_with_collision_check(path=path, navigable_map=navigable_map)
        simplified_path.pop(0)
        final_path = []
        s = self._px_to_xy(start)
        for e in simplified_path:
            e = self._px_to_xy(e)
            sampled_points = self._sample_points_between_two_points(s.squeeze(), e.squeeze())
            final_path.extend(sampled_points)
            s = e

        # 8. Visualize both paths for comparison
        if self.task_config['verbose']:
            plt.figure(figsize=(10, 10))
            plt.imshow(navigable_map, cmap='gray')
            plt.scatter(
                [start_point[0], goal_point[0]],
                [start_point[1], goal_point[1]],
                color='red',
                marker='o',
                s=10,
            )
            plt.title('Navigable Map')
            plt.savefig(os.path.join(self.task_config['agent_path'], 'images/map.jpg'))

            plt.figure(figsize=(10, 10))
            plt.imshow(navigable_map, cmap='gray')
            plt.scatter([start[0], goal[0]], [start[1], goal[1]], color='red', marker='o', s=10)
            plt.title('Navigable Map')
            plt.savefig(os.path.join(self.task_config['agent_path'], 'images/map_modi.jpg'))

            _final_path = [tuple(self._xy_to_px(point[:2]).squeeze()) for point in final_path]
            plt.figure(figsize=(10, 10))
            plt.imshow(navigable_map, cmap='gray')
            original_x, original_y = zip(*path)
            simplified_x, simplified_y = zip(*_final_path)
            plt.plot(original_x, original_y, 'b-', label='Original Path')
            plt.plot(simplified_x, simplified_y, 'r-', label='Simplified Path')
            plt.scatter(
                [start[0], goal[0]],
                [start[1], goal[1]],
                color='green',
                label='Start/Goal',
            )
            plt.legend()
            plt.title('Comparison of Original and Simplified Paths')
            plt.savefig(os.path.join(self.task_config['agent_path'], 'images/comparison.jpg'))
        return final_path

    def _sample_points_between_two_points(self, start, end, step=1, z=1.05):
        x1, y1 = start
        x2, y2 = end
        distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        unit_vector = ((x2 - x1) / distance, (y2 - y1) / distance)

        num_samples = int(distance / step)
        sampled_points = [
            [x2 - n * step * unit_vector[0], y2 - n * step * unit_vector[1], z] for n in range(num_samples + 1)
        ]
        sampled_points.reverse()

        return sampled_points

    def _px_to_xy(self, points: np.ndarray) -> np.ndarray:
        px = np.array(points).reshape(-1, 2)
        px_copy = px.copy()
        px_copy[:, 0] = self.map_config['size'] - px_copy[:, 0]
        points = (px_copy - np.array([self.map_config['size'] // 2, self.map_config['size'] // 2])) / self.map_config[
            'pixels_per_meter'
        ]
        return points[:, ::-1]

    def _xy_to_px(self, points: np.ndarray) -> np.ndarray:
        points = np.array(points).reshape(-1, 2)
        px = np.rint(points[:, ::-1] * self.map_config['pixels_per_meter']) + np.array(
            [self.map_config['size'] // 2, self.map_config['size'] // 2]
        )
        px[:, 0] = self.map_config['size'] - px[:, 0]
        return px
