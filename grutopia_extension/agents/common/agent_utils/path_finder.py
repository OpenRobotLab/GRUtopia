import time
from collections import deque
from typing import List

import matplotlib.pyplot as plt
import numpy as np
from pathfinding.core.grid import Grid
from pathfinding.core.heap import SimpleHeap
from pathfinding.core.node import GridNode
from pathfinding.core.util import backtrace
from pathfinding.finder.a_star import AStarFinder
from PIL import Image
from scipy.ndimage import distance_transform_edt


class TimeLimitedBiAStarFinder(AStarFinder):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def keep_running(self):
        """
        check, if we run into time or iteration constrains.
        :returns: True if we keep running and False if we run into a constraint
        """
        if self.runs >= self.max_runs:
            print(
                '{} run into barrier of {} iterations without '
                'finding the destination'.format(self.__class__.__name__, self.max_runs)
            )
            return False

        if time.time() - self.start_time >= self.time_limit:
            print('{} took longer than {} seconds, aborting!'.format(self.__class__.__name__, self.time_limit))
            return False
        return True

    def find_path(self, start: GridNode, end: GridNode, grid: Grid):
        """
        find a path from start to end node on grid using the A* algorithm
        :param start: start node
        :param end: end node
        :param grid: graph or grid that stores all possible nodes
        :return:
        """
        start.g = 0
        start.f = 0

        self.clean_grid(grid)

        self.start_time = time.time()  # execution time limitation
        self.runs = 0  # count number of iterations
        start.opened = True

        open_list = SimpleHeap(start, grid)

        running = True
        while len(open_list) > 0 and running:
            self.runs += 1
            running = self.keep_running()

            path = self.check_neighbors(start, end, grid, open_list)
            if path:
                return path

        # failed to find path, return the nearest
        nodes = []
        for line_nodes in grid.nodes:
            nodes.extend([node for node in line_nodes if node.closed])
        nearest_node = min(nodes, key=lambda node: (node.x - end.x) ** 2 + (node.y - end.y) ** 2)
        path = backtrace(nearest_node)
        return path


def find_nearest_reset_point(current_point: tuple, freemap: np.ndarray, distance_threshold: int = 5) -> tuple:
    """
    Find the nearest free point from the current point that is at a distance greater
    than the specified threshold from any obstacle.

    Parameters:
    - freemap: A 2D numpy array representing the free space map. 1 represents free space, 0 represents obstacles.
    - current_point: The coordinates of the current point (x, y).
    - distance_threshold: The minimum distance from obstacles.

    Returns:
    - (x, y): The coordinates of the nearest free point that meets the condition.
    """
    # Compute the distance transform, which returns the distance of each free point to the nearest obstacle
    distance_map = distance_transform_edt(freemap)

    # Find all points with distance greater than the distance_threshold
    valid_points = np.argwhere(distance_map > distance_threshold)

    if len(valid_points) == 0:
        return None  # No points satisfy the condition

    # Compute the Euclidean distance from all valid_points to the current_point
    distances_to_current = np.linalg.norm(valid_points - (current_point[1], current_point[0]), axis=1)

    # Find the point with the minimum distance
    nearest_point_idx = np.argmin(distances_to_current)
    nearest_point = tuple(valid_points[nearest_point_idx])
    return (nearest_point[1], nearest_point[0])


def find_nearest_free_point(point: tuple, navigable_map: np.ndarray) -> np.ndarray:
    """
    Finds the nearest navigable (walkable) point to the given coordinates using BFS.
    If the point is in an occupied area, it searches adjacent points until it finds a free one.

    Parameters:
        point (tuple): The (x, y) coordinates of the starting point.
        navigable_map (numpy.ndarray): 2D array representing the navigable map where
                                       1 indicates free space and 0 indicates obstacles.

    Returns:
        tuple: The (x, y) coordinates of the nearest free point, or None if none is found.
    """
    x, y = point
    max_rows, max_cols = navigable_map.shape

    # Check if the starting point is within the map bounds
    if not (0 <= x < max_cols and 0 <= y < max_rows):
        raise ValueError('Starting point is outside the map boundaries.')

    # If the starting point is already free, return it
    if navigable_map[y, x] == 1:
        return (x, y)

    # Initialize BFS
    visited = set()
    queue = deque()
    queue.append((x, y))

    # Define possible movements (8-connected grid)
    movements = [
        (-1, 0),  # Left
        (1, 0),  # Right
        (0, -1),  # Up
        (0, 1),  # Down
        (-1, -1),  # Up-Left
        (-1, 1),  # Down-Left
        (1, -1),  # Up-Right
        (1, 1),
    ]  # Down-Right

    while queue:
        current_x, current_y = queue.popleft()

        # Skip if already visited
        if (current_x, current_y) in visited:
            continue
        visited.add((current_x, current_y))

        # Check if current position is within bounds
        if not (0 <= current_x < max_cols and 0 <= current_y < max_rows):
            continue

        # Check if current position is navigable
        if navigable_map[current_y, current_x] == 1:
            return (current_x, current_y)

        # Enqueue adjacent positions
        for dx, dy in movements:
            neighbor_x = current_x + dx
            neighbor_y = current_y + dy

            if 0 <= neighbor_x < max_cols and 0 <= neighbor_y < max_rows and (neighbor_x, neighbor_y) not in visited:
                queue.append((neighbor_x, neighbor_y))

    # If no free point is found, return None
    return None


def is_line_collision_free(navigable_map: np.ndarray, point1: tuple, point2: tuple) -> bool:
    """
    Checks if the line between two points is collision-free using NumPy vectorization.

    Parameters:
        navigable_map (numpy.ndarray): 2D array representing the map (0: obstacle, 1: free space).
        point1 (tuple of float): Starting point coordinates (x0, y0).
        point2 (tuple of float): Ending point coordinates (x1, y1).

    Returns:
        bool: True if the line is collision-free, False if it intersects any obstacles.
    """
    x0, y0 = point1
    x1, y1 = point2

    x0, y0 = float(x0), float(y0)
    x1, y1 = float(x1), float(y1)

    num_points = int(max(abs(x1 - x0), abs(y1 - y0))) + 1

    x_values = np.linspace(x0, x1, num_points)
    y_values = np.linspace(y0, y1, num_points)

    x_indices = np.clip(np.round(x_values).astype(int), 0, navigable_map.shape[1] - 1)
    y_indices = np.clip(np.round(y_values).astype(int), 0, navigable_map.shape[0] - 1)

    # Check if any point along the line is an obstacle
    collision = (navigable_map[y_indices, x_indices] == 0).any()

    return not collision


def simplify_path_with_collision_check(
    path: List[tuple], navigable_map: np.ndarray, epsilon: float = 5.0
) -> List[tuple]:
    """
    Simplifies the path using the Ramer-Douglas-Peucker algorithm with collision checking.

    Parameters:
        path (list of tuple): Original path as a list of (x, y) coordinates.
        navigable_map (numpy.ndarray): 2D array representing the map.
        epsilon (float): Maximum allowed deviation from the original path.

    Returns:
        list of tuple: Simplified path.
    """

    def rdp(start_idx, end_idx, path, simplified):
        if start_idx >= end_idx:
            return
        # Find the point with the maximum distance to the line
        max_dist = 0.0
        index = start_idx
        line_start = np.array(path[start_idx])
        line_end = np.array(path[end_idx])
        for i in range(start_idx + 1, end_idx):
            point = np.array(path[i])
            dist = np.abs(np.cross(line_end - line_start, line_start - point)) / np.linalg.norm(line_end - line_start)
            if dist > max_dist:
                max_dist = dist
                index = i
        # If max distance is greater than epsilon, recurse
        if max_dist > epsilon:
            rdp(start_idx, index, path, simplified)
            simplified.append(path[index])
            rdp(index, end_idx, path, simplified)
        else:
            # Check for collision between start and end points
            if not is_line_collision_free(navigable_map, path[start_idx], path[end_idx]):
                mid_idx = (start_idx + end_idx) // 2
                rdp(start_idx, mid_idx, path, simplified)
                simplified.append(path[mid_idx])
                rdp(mid_idx, end_idx, path, simplified)
            # Else, the segment is acceptable and no action is needed

    if not path:
        return []

    simplified_path = [path[0]]
    rdp(0, len(path) - 1, path, simplified_path)
    simplified_path.append(path[-1])

    # Sort the simplified path based on original indices to maintain order
    simplified_path = sorted(set(simplified_path), key=lambda x: path.index(x))

    return simplified_path


def main():
    from pathfinding.core.diagonal_movement import DiagonalMovement

    # 1. Load or create the navigable map
    # For demonstration, we'll create a sample map
    # 0 represents obstacles, 1 represents navigable space
    navigable_map = Image.open('images/navigatable_map.jpg').convert('L')
    threshold = 128
    binary_image = navigable_map.point(lambda x: 1 if x > threshold else 0, '1')
    navigable_map = np.array(binary_image, dtype=np.uint8)

    # 2. Define the start and goal points
    goal_point = (550, 702)  # Replace with your start point
    start_point = (400, 755)  # Replace with your goal point
    # goal_point = (475, 755)  # Replace with your goal point

    # Visualize the navigable map
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
    plt.savefig('map.jpg')

    # 3. Find the nearest free points to start and goal
    start = find_nearest_free_point(start_point, navigable_map)
    goal = find_nearest_free_point(goal_point, navigable_map)

    if start is None or goal is None:
        print('Cannot find valid start or goal point.')
        return

    print(f'Adjusted Start Point: {start}')
    print(f'Adjusted Goal Point: {goal}')

    # Visualize the navigable map
    plt.figure(figsize=(10, 10))
    plt.imshow(navigable_map, cmap='gray')
    plt.scatter([start[0], goal[0]], [start[1], goal[1]], color='red', marker='o', s=10)
    plt.title('Navigable Map')
    plt.savefig('map_modi.jpg')

    # 4. Prepare the grid for pathfinding
    matrix = navigable_map.tolist()
    grid = Grid(matrix=matrix)

    # 5. Set up the start and end nodes
    grid_start = grid.node(start[0], start[1])
    grid_end = grid.node(goal[0], goal[1])

    # 6. Perform A* pathfinding
    finder = TimeLimitedBiAStarFinder(time_limit=10, diagonal_movement=DiagonalMovement.always)
    begin = time.time()
    path, _ = finder.find_path(grid_start, grid_end, grid)
    print(time.time() - begin)

    if path and path[-1] == grid_end:
        print('Path found to the goal.')
    else:
        print('No path to the goal found within the time limit.')
        print('Returning path to the closest point.')

    print(f'Original Path Length: {len(path)}')

    # Visualize the original path
    navigable_map_visual = navigable_map.copy()
    for x, y in path:
        navigable_map_visual[y][x] = 0.5  # Mark the path

    plt.figure(figsize=(10, 10))
    plt.imshow(navigable_map_visual, cmap='gray')
    plt.title('Original Path')
    plt.savefig('ori_path.jpg')

    # 7. Simplify the path with collision checking
    path = [(point.x, point.y) for point in path]
    epsilon = 5.0  # Adjust epsilon as needed
    simplified_path = simplify_path_with_collision_check(path=path, epsilon=epsilon, navigable_map=navigable_map)

    print(f'Simplified Path Length: {len(simplified_path)}')

    # Visualize the simplified path
    navigable_map_visual_simplified = navigable_map.copy()
    for x, y in simplified_path:
        navigable_map_visual_simplified[y][x] = 0.5  # Mark the simplified path

    plt.figure(figsize=(10, 10))
    plt.imshow(navigable_map_visual_simplified, cmap='gray')
    plt.title('Simplified Path')
    plt.savefig('sim_path.jpg')

    # 8. Visualize both paths for comparison
    plt.figure(figsize=(10, 10))
    plt.imshow(navigable_map, cmap='gray')
    original_x, original_y = zip(*path)
    simplified_x, simplified_y = zip(*simplified_path)
    plt.plot(original_x, original_y, 'b-', label='Original Path')
    plt.plot(simplified_x, simplified_y, 'r-', label='Simplified Path')
    plt.scatter([start[0], goal[0]], [start[1], goal[1]], color='green', label='Start/Goal')
    plt.legend()
    plt.title('Comparison of Original and Simplified Paths')
    plt.savefig('comparison.jpg')


if __name__ == '__main__':
    main()
