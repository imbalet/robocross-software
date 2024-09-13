import numpy as np

import heapq

from utils import polar_to_decart

import cv2


class Astar():
    NON_WALKABLE_WEIGHT = 255
    
    class Node:
        def __init__(self, position:tuple, cost:float, heuristic:float, dir:float, parent:"Astar.Node" = None):
            self.position = position
            self.cost = cost
            self.heuristic = heuristic
            self.parent = parent
            self.dir = dir

        def f(self):
            return self.cost + self.heuristic

        def __lt__(self, other):
            return self.f() < other.f()


    def is_in_goal(self, start:tuple, goal:tuple) -> bool:
        return ((pow(start[0] - goal[0], 2) + pow(start[1] - goal[1], 2)) ** 0.5) <= self.goalRad
    
    
    def get_neighbours(self, matrix, node:"Astar.Node", goal, closed_list):
        neighbours = []
        for r, th, p_w in self.both_neighbours:
            th = node.dir + th
            x, y = polar_to_decart(r, th)
            x, y = round(x), round(y)
            
            node_pos = (node.position[0] - y, node.position[1] + x)
            if (node_pos, th) in closed_list or not self.check_collision(matrix, node_pos): continue
            if ( 0 < node_pos[0] < len(matrix)) and (0 < node_pos[1] < len(matrix[0])) and matrix[node_pos[0]][node_pos[1]] != Astar.NON_WALKABLE_WEIGHT:
                p_w += matrix[node_pos[0], node_pos[1]]
                neighbour = Astar.Node((node.position[0] - y, node.position[1] + x), p_w, Astar.heuristic(node_pos, goal), th, node)
                neighbours.append(neighbour)
        return neighbours

    @staticmethod
    def heuristic(a, b) -> float:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)**0.5
    
    
    def check_collision(self, grid:np.ndarray, pose:tuple):
        try:
            # Определяем границы области проверки
            y_start = int(pose[0] - self.robotRad)
            y_end = int(pose[0] + self.robotRad)
            x_start = int(pose[1] - self.robotRad)
            x_end = int(pose[1] + self.robotRad)

            # Извлекаем область интереса из сетки
            col = np.copy(grid[y_start:y_end, x_start:x_end])

            # Создаем маску круга
            yy, xx = np.ogrid[:col.shape[0], :col.shape[1]]
            mask = (xx - self.robotRad) ** 2 + (yy - self.robotRad) ** 2 <= self.robotRad ** 2

            # Проверяем, есть ли коллизии
            if np.any(col[mask] == self.NON_WALKABLE_WEIGHT):
                return False  # Найдена коллизия (значение больше порога)
            return True  # Нет коллизий
        except Exception as e:
            print(e)
            return False
    
    
    def __init__(self, robot_radius:float, goal_radius:float, car_steering:float, 
                 path_discrete:float):
        self.robotRad = robot_radius
        self.goalRad = goal_radius
        self.carSteer = car_steering
        self.pathDisc = path_discrete
        
        self.both_neighbours = [
            (self.pathDisc, self.carSteer, 1.4),
            (self.pathDisc, 0.0, 1.0),
            (self.pathDisc, -self.carSteer, 1.4),
            # (-self.pathDisc, self.carSteer, 14.0),
            # (-self.pathDisc, 0.0, 10.0),
            # (-self.pathDisc, -self.carSteer, 14.0),
        ]
    

    def astar(self, matrix, start:tuple[int, int, float], goal:tuple[int, int]):
        """
        Функция поиска пути методом A*        
        
        Attributes
        ----------
        matrix : np.ndarray 
            матрица клеток
            
        start : tuple[int, int, float]
            кортеж вида (row, collumn, orientation) представляющий 
            положение робота в сетке и его ориентацию
        """
        
        param = 0
        
        if matrix[start[0], start[1]] >= 127 or matrix[goal[0], goal[1]] >= 127:
            return None  # Начальная или конечная клетка занята

        open_list : list[Astar.Node] = []
        closed_list = set()

        start_node = Astar.Node(start, 0, Astar.heuristic(start, goal), start[2])
        heapq.heappush(open_list, start_node)

        while open_list:
            current_node = heapq.heappop(open_list)
            closed_list.add((current_node.position, current_node.dir))

            if self.is_in_goal(current_node.position, goal):
                path = []
                while current_node:
                    path.append((*current_node.position[:2], current_node.dir))
                    current_node = current_node.parent
                if param:
                    self.viz(matrix, path[::-1])
                return path[::-1]  # Путь в правильном порядке


            neighbors:list["Astar.Node"] = self.get_neighbours(matrix, current_node, goal, closed_list)

            for node in neighbors:
                
                for open_node in open_list:
                    if open_node.position == node.position and node.cost >= open_node.cost:
                        break
                else:
                    heapq.heappush(open_list, node)

        return None  # Если путь не найден

    def viz(self, matrix, path):
        a= np.copy(matrix)
        for i in range(len(path) - 1):
            p = path[i]
            p1 = path[i + 1]
            # cv2.circle(a, [p[1], p[0]], 2, [255], -1)
            cv2.line(a, (p[1], p[0]), (p1[1], p1[0]), [255] )
        cv2.imshow('Image', a)
        cv2.waitKey(0)
        
        


if __name__ == "__main__":
    import os
    import json

    os.chdir(os.path.dirname(__file__))
    with open("m.json") as f:
        a = json.load(f)

    # a = np.zeros((100, 100), dtype=np.uint8)
    # cv2.circle(a, [50, 50], 40, [70], -1)

    # a[20, 79] = 255

    # cv2.circle(a, [79, 24], 9, [255], -1)

    a = np.array(a, np.uint8)

    finder = Astar(15, 20, 0.3, 15)

    path = finder.astar(a, (400, 399, 0), (301, 606))
    cv2.circle(a, [399, 400], 9, [255], -1)
    cv2.circle(a, [606, 301], 9, [255], -1)

    cv2.imshow('Image', a)
    cv2.waitKey(0)    

    # path = finder.get_path(gr, [15, 15, 0], [85, 85])
    if type(path) is list:
        for i in range(len(path) - 1):
            p = path[i]
            p1 = path[i + 1]
            # cv2.circle(a, [p[1], p[0]], 2, [255], -1)
            cv2.line(a, (p[1], p[0]), (p1[1], p1[0]), [255] )
    else:
        print(path)

    print(path)
        


    cv2.imshow('Image', a)
    cv2.waitKey(0)