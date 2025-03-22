import numpy as np
import heapq
from utils import polar_to_decart
import cv2


class Astar:
    NON_WALKABLE_WEIGHT = 255

    class Node:
        __slots__ = ("position", "cost", "heuristic", "parent", "dir")

        def __init__(
            self,
            position: tuple,
            cost: float,
            heuristic: float,
            dir: float,
            parent: "Astar.Node" = None,
        ):
            self.position = position
            self.cost = cost
            self.heuristic = heuristic
            self.parent = parent
            self.dir = dir

        def f(self):
            return self.cost + self.heuristic

        def __lt__(self, other):
            return self.f() < other.f()

    def __init__(
        self,
        robot_radius: float,
        goal_radius: float,
        car_steering: float,
        path_discrete: float,
    ):
        self.robotRad = int(robot_radius)
        self.goalRad = goal_radius
        self.carSteer = car_steering
        self.pathDisc = path_discrete

        self.both_neighbours = [
            (self.pathDisc, self.carSteer, 1.4),
            (self.pathDisc, 0.0, 1.0),
            (self.pathDisc, -self.carSteer, 1.4),
        ]

    def is_in_goal(self, current_pos: tuple, goal: tuple) -> bool:
        dx = current_pos[0] - goal[0]
        dy = current_pos[1] - goal[1]
        return dx**2 + dy**2 <= (self.goalRad**2)

    def get_neighbours(self, matrix, node: "Astar.Node", goal, closed_list: set):
        neighbours = []
        for r, th_add, p_w in self.both_neighbours:
            th = node.dir + th_add
            dx, dy = polar_to_decart(r, th)
            dx, dy = round(dx), round(dy)
            node_pos = (node.position[0] - dy, node.position[1] + dx)

            if (node_pos, th) in closed_list:
                continue

            if not self.check_collision(matrix, node_pos):
                continue

            new_cost = node.cost + p_w + matrix[node_pos[0], node_pos[1]]
            heuristic = self.heuristic(node_pos, goal)
            neighbours.append(Astar.Node(node_pos, new_cost, heuristic, th, node))
        return neighbours

    @staticmethod
    def heuristic(a, b) -> float:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def check_collision(self, grid: np.ndarray, pose: tuple) -> bool:
        y, x = pose[0], pose[1]
        if y < 0 or y >= grid.shape[0] or x < 0 or x >= grid.shape[1]:
            return False

        y_start = max(0, y - self.robotRad)
        y_end = min(grid.shape[0], y + self.robotRad + 1)
        x_start = max(0, x - self.robotRad)
        x_end = min(grid.shape[1], x + self.robotRad + 1)

        if y_start >= y_end or x_start >= x_end:
            return False

        col_region = grid[y_start:y_end, x_start:x_end]
        center_y = y - y_start
        center_x = x - x_start

        yy, xx = np.ogrid[: col_region.shape[0], : col_region.shape[1]]
        mask = (xx - center_x) ** 2 + (yy - center_y) ** 2 <= self.robotRad**2

        return not np.any(col_region[mask] == self.NON_WALKABLE_WEIGHT)

    def astar(self, matrix, start: tuple[int, int, float], goal: tuple[int, int]):
        if matrix[start[0], start[1]] >= 127 or matrix[goal[0], goal[1]] >= 127:
            return None

        open_heap = []
        closed_set = set()
        cost_so_far = {}

        start_node = Astar.Node(
            (start[0], start[1]), 0.0, self.heuristic(start, goal), start[2]
        )
        heapq.heappush(open_heap, start_node)
        start_key = (start_node.position, start_node.dir)
        cost_so_far[start_key] = start_node.cost

        while open_heap:
            current_node = heapq.heappop(open_heap)
            current_key = (current_node.position, current_node.dir)

            if (
                current_key not in cost_so_far
                or cost_so_far[current_key] < current_node.cost
            ):
                continue

            if self.is_in_goal(current_node.position, goal):
                path = []
                while current_node:
                    path.append((*current_node.position, current_node.dir))
                    current_node = current_node.parent
                return path[::-1]

            closed_set.add(current_key)

            neighbors = self.get_neighbours(matrix, current_node, goal, closed_set)
            for neighbor in neighbors:
                neighbor_key = (neighbor.position, neighbor.dir)
                if neighbor_key in closed_set:
                    continue

                if (
                    neighbor_key in cost_so_far
                    and cost_so_far[neighbor_key] <= neighbor.cost
                ):
                    continue

                cost_so_far[neighbor_key] = neighbor.cost
                heapq.heappush(open_heap, neighbor)

        return None

    def viz(self, matrix, path):
        img = np.copy(matrix)
        for i in range(len(path) - 1):
            p = path[i]
            p1 = path[i + 1]
            cv2.line(img, (p[1], p[0]), (p1[1], p1[0]), [255])
        cv2.imshow("Path", img)
        cv2.waitKey(0)


if __name__ == "__main__":
    import json
    import os

    os.chdir(os.path.dirname(__file__))
    with open("m.json") as f:
        grid_data = json.load(f)

    grid = np.array(grid_data, dtype=np.uint8)
    path_finder = Astar(
        robot_radius=15, goal_radius=20, car_steering=0.3, path_discrete=15
    )

    start = (400, 399, 0)
    goal = (301, 606)
    path = path_finder.astar(grid, start, goal)

    cv2.circle(grid, (start[1], start[0]), 9, 255, -1)
    cv2.circle(grid, (goal[1], goal[0]), 9, 255, -1)

    if path:
        path_finder.viz(grid, path)
    else:
        print("Path not found")
    cv2.destroyAllWindows()
