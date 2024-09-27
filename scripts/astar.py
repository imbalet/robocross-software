#! /bin/python3

import numpy as np

import heapq
import math

import cv2

# from test import is_collision, draw_filled_tilted_rectangle
# from test_finding import find_nearest

from reeds_shepp import calc_optimal_path 
# from curves import calc_all_paths

import matplotlib.pyplot as plt

def show(img):
    plt.figure(figsize=(10, 10))
    
    # Отображаем изображение
    plt.imshow(img)
    
    # Убираем оси
    plt.axis('off')
    
    # Показываем изображение
    plt.show()


def polar_to_decart(rho, phi):
    return rho * np.cos(phi), rho * np.sin(phi)


def decart_to_polar(x, y):
    return np.sqrt(x ** 2 + y ** 2), np.arctan2(y, x)

class Astar():
    NON_WALKABLE_WEIGHT = 127
    
    class Node:
        def __init__(self, position:tuple, cost:float, heuristic:float, dir:float, direction, parent:"Astar.Node" = None):
            self.position = position
            self.cost = cost
            self.heuristic = heuristic
            self.parent = parent
            self.dir = dir
            self.direction = direction

        def f(self):
            return self.cost + self.heuristic

        def __lt__(self, other):
            return self.f() < other.f()



    def __init__(self, robot_size:tuple[float, float], goal_radius:float, car_steering:float, 
                 path_discrete:float):
        
        self.robot_w = robot_size[0]
        self.robot_h = robot_size[1]

        self.robot_ellipse_axes = int(self.robot_w / 2 * 1.4), int(self.robot_h / 2 * 1.4)
        self.elipse_rad = 1 + int(( 2 * ((max(self.robot_w, self.robot_h) / 2) ** 2)) ** 0.5)
        k = self.elipse_rad / min(self.robot_ellipse_axes)
        self.mask_ellipse_axes = int(self.robot_ellipse_axes[0] * k), int(self.robot_ellipse_axes[1] * k)
        
        self.elipse_th = max(self.mask_ellipse_axes) - max(self.robot_ellipse_axes)
        
        self.area_size = int(math.hypot(*robot_size)) + 1
        self.mask_rect_th = self.area_size - min(robot_size)
        self.mask_rect_size = (robot_size[0] + self.mask_rect_th, robot_size[1] + self.mask_rect_th)
            
        self.goalRad = goal_radius
        self.carSteer = car_steering
        self.pathDisc = path_discrete
        
        # self.carSteer = math.asin(self.pathDisc / (2 * 50)) 
        
        self.both_neighbours = [
            (self.pathDisc, self.carSteer, 1.4),
            (self.pathDisc, 0.0, 1.0),
            (self.pathDisc, -self.carSteer, 1.4),
            (-self.pathDisc, self.carSteer, 24.0),
            (-self.pathDisc, 0.0, 20.0),
            (-self.pathDisc, -self.carSteer, 24.0),
        ]


    def is_in_goal(self, start:tuple, dir:float, goal:tuple) -> bool:
        return abs(dir - goal[2]) < 0.7 and ((pow(start[0] - goal[0], 2) + pow(start[1] - goal[1], 2)) ** 0.5) <= self.goalRad
        # abs(dir - goal[2]) < 0.3 and
    
    
    def get_neighbours(self, matrix, node:"Astar.Node", goal, closed_list):
        neighbours = []
        for r, th, p_w in self.both_neighbours:
            th = node.dir + th
            x, y = polar_to_decart(r, th)
            x, y = round(x), round(y)
            
            node_pos = (node.position[0] - y, node.position[1] + x)
            if (node_pos, th) in closed_list or not self.check_collision(matrix, node_pos, th): continue
            if ( 0 < node_pos[0] < len(matrix)) and (0 < node_pos[1] < len(matrix[0])) and matrix[node_pos[0]][node_pos[1]] != Astar.NON_WALKABLE_WEIGHT:
                p_w += matrix[node_pos[0], node_pos[1]]
                neighbour = Astar.Node((node.position[0] - y, node.position[1] + x), p_w, Astar.heuristic(node_pos, goal), th, 1 if r > 0 else -1, node)
                neighbours.append(neighbour)
        return neighbours

    @staticmethod
    def heuristic(a, b) -> float:
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)**0.5
    
    
    @staticmethod    
    def __draw_tilted_rectangle(image, center, width, height, angle, th, color=126):
        rectangle = ((center[1], center[0]), (height, width), -angle * 57.3)

        box_points = cv2.boxPoints(rectangle)
        box_points = np.intp(box_points)
        cv2.polylines(image.copy(), [box_points], isClosed=True, color=color, thickness=th)

    
    
    def check_collision(self, grid, node:tuple[int, int], angle):
        try:
            col_t = np.copy(grid[int(node[0] - self.area_size // 2):int(node[0] + self.area_size // 2), 
                              int(node[1] - self.area_size // 2):int(node[1] + self.area_size // 2)])
            
            Astar.__draw_tilted_rectangle(col_t, (self.area_size // 2, self.area_size // 2), *self.mask_rect_size, angle, self.mask_rect_th, 120)
            # show(col_t)
        except cv2.error as e:
            print(e)
            return False
        try:
            if np.max(col_t.data) == 127:
                return False
            else:
                return True
        except Exception as e:
            print(e)
            return False 
    
    
    # def check_collision1(self, grid:np.ndarray, pose:tuple, angle):
    #     obs = find_nearest(grid, pose, 255, max(self.robot_h, self.robot_w))
    #     if obs is not None:
    #         return not is_collision(pose, (self.robot_w, self.robot_h), angle, obs)
    #     return True

    # def check_collision3(self, grid, node:tuple[int, int], angle):
    #     try:
    #         col_t = np.copy(grid[int(node[0] - self.elipse_rad):int(node[0] + self.elipse_rad), 
    #                           int(node[1] - self.elipse_rad):int(node[1] + self.elipse_rad)])
            
    #         cv2.ellipse(col_t, (self.elipse_rad, self.elipse_rad), self.mask_ellipse_axes, -angle * 57.3 + 90, 0, 360, 127, self.elipse_th)

    #         show(col_t)
            
    #     except cv2.error as e:
    #         print(e)
    #         return False
    #     try:
    #         if np.max(col_t.data) == 255:
    #             return False
    #         else:
    #             return True
    #     except Exception as e:
    #         print(e)
    #         return False    
    
    
    # def check_collision2(self, grid:np.ndarray, pose:tuple):
    #     try:
    #         # Определяем границы области проверки
    #         y_start = int(pose[0] - self.robotRad)
    #         y_end = int(pose[0] + self.robotRad)
    #         x_start = int(pose[1] - self.robotRad)
    #         x_end = int(pose[1] + self.robotRad)

    #         # Извлекаем область интереса из сетки
    #         col = np.copy(grid[y_start:y_end, x_start:x_end])

    #         # Создаем маску круга
    #         yy, xx = np.ogrid[:col.shape[0], :col.shape[1]]
    #         mask = (xx - self.robotRad) ** 2 + (yy - self.robotRad) ** 2 <= self.robotRad ** 2

    #         # Проверяем, есть ли коллизии
    #         if np.any(col[mask] == self.NON_WALKABLE_WEIGHT):
    #             return False  # Найдена коллизия (значение больше порога)
    #         return True  # Нет коллизий
    #     except Exception as e:
    #         print(e)
    #         return False
    
    
    def reconstruct_path(self, node):
        path = []
        while node:
            path.append((*node.position[:2], node.dir, node.direction))
            node = node.parent
        return path[::-1]  # Путь в правильном порядке
    
    
    def reeds_shepp(self, matrix, current_node, goal, turn_radius):
        # paths = calc_all_paths(*start, *goal, 1 / turn_rad, discrete)
            
        def inds2coords(ind, shape = matrix.shape):
            return ind[1], shape[0] - ind[0]
        def coords2inds(coord, shape = matrix.shape):
            return shape[0] - coord[1], coord[0]
        
        # paths = calc_all_paths(*inds2coords(current_node.position), current_node.dir, *inds2coords(goal), goal[2], 1 / (turn_radius * 2), self.pathDisc)
        path = calc_optimal_path(*inds2coords(current_node.position), current_node.dir, *inds2coords(goal), goal[2], 1 / (turn_radius * 2), self.pathDisc)
        if path:
            p = []
            for i in path:
                if self.check_collision(matrix, coords2inds((int(i[0]), int(i[1]))), i[2]):
                    p.append((*coords2inds((int(i[0]), int(i[1]))), i[2], i[3]))
                else:
                    print("path in collission")
                    break
            else:
                astar_path = self.reconstruct_path(current_node)
                print("path found!")
                return astar_path + p
        else:
            print("paths not found")  
        return None  
    

    def astar(self, matrix, start:tuple[int, int, float], goal:tuple[int, int, float], turn_radius = 10,  reeds_shepp_dist = 80):
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
        
        counter = 0
        
        if matrix[start[0], start[1]] >= 127 or matrix[goal[0], goal[1]] >= 127:
            return None  # Начальная или конечная клетка занята

        open_list : list[Astar.Node] = []
        closed_list = set()

        start_node = Astar.Node(start, 0, Astar.heuristic(start, goal), start[2], 1)
        heapq.heappush(open_list, start_node)

        while open_list:
            if counter >= 400: return None
            counter += 1
            current_node = heapq.heappop(open_list)
            closed_list.add((current_node.position, current_node.dir))

            if self.is_in_goal(current_node.position, current_node.dir, goal):
                return self.reconstruct_path(current_node)

            
            if Astar.heuristic(current_node.position, goal) <= reeds_shepp_dist:
                # path = self.reeds_shepp(matrix, current_node, goal, turn_radius)
                
                # # if path is not None:
                # #     return path
                
                # print("trying find reeds sheppp path")
                
                def inds2coords(ind, shape = matrix.shape):
                    return ind[1], shape[0] - ind[0]
                def coords2inds(coord, shape = matrix.shape):
                    return shape[0] - coord[1], coord[0]
                
                path = calc_optimal_path(*inds2coords(current_node.position), current_node.dir, *inds2coords(goal), goal[2], 1 / (turn_radius * 2), self.pathDisc)
                if path:
                    p = []
                    for i in path:
                        if self.check_collision(matrix, coords2inds((int(i[0]), int(i[1]))), i[2]):
                            p.append((*coords2inds((int(i[0]), int(i[1]))), i[2], i[3]))
                        else:
                            # print("path in collission")
                            break
                    else:
                        astar_path = self.reconstruct_path(current_node)
                        return astar_path + p
                        print("path found!")
                else:
                    # print("path not found")
                    pass
                
                        
                

            neighbors:list["Astar.Node"] = self.get_neighbours(matrix, current_node, goal, closed_list)

            for node in neighbors:
                
                for open_node in open_list:
                    if open_node.position == node.position and open_node.dir == node.dir and node.cost >= open_node.cost:
                        break
                else:
                    heapq.heappush(open_list, node)

        return None  # Если путь не найден



def draw_tilted_rectangle(image, center, width, height, rad):
    rectangle = ((center[0], center[1]), (width, height), rad * 57.3)
    box_points = cv2.boxPoints(rectangle)
    box_points = np.int0(box_points)
    cv2.polylines(image, [box_points], isClosed=True, color=(255, 255, 255), thickness=2)




def parking():
    # 99, 319
    img = cv2.imread('d.png')
    a = img[:, :, 0].astype(np.uint8)
    
    finder = Astar((17, 45), 10, 0.3, 15)
    
    # path = finder.astar(a, (84, 218, 0), (84, 354, 3.14/2), 50)
    # path = finder.astar(a, (84, 354, 0), (113, 271, 3.14/2), 50)
    # path += finder.astar(a, (172, 202, 3.14/2), (249, 202, 3.14/2), 100)
    path = finder.astar(a, (84, 218, 0), (84, 364, 0), 50)
    path += finder.astar(a, (84, 364, 0), (94, 307, 3.14/5 ), 50)
    path += finder.astar(a, (94, 307, 3.14/5), (113, 271, -0.1), 50)
    
    cv2.imshow('Image', a)
    cv2.waitKey(0)    

    if type(path) is list:
        for p in path[::1]:
            draw_filled_tilted_rectangle(a, (p[0], p[1]), 17, 43, p[2])
            cv2.circle(a, [p[1], p[0]], 2, [255], -1)
            cv2.imshow('Image', a)
            cv2.waitKey(0)
    else:
        print(path)
    print(path)    
    cv2.imshow('Image', a)
    cv2.waitKey(0)


def coords_to_inds(coords, shape):
    row = shape[0] // 2 - coords[1]
    col = shape[1] // 2 + coords[0]
    return row, col


def inds_to_coords(inds, shape):
    y = shape[0] // 2 - inds[0]
    x = -shape[1] // 2 + inds[1]
    return x, y


def parking1():
    
    img = cv2.imread('b.png')
    a = img[:, :, 0].astype(np.uint8)
    # a = np.zeros(a.shape, a.dtype)
    
    
    finder = Astar((17, 45), 10, 0.3, 10)
    # path = finder.astar(a, (209, 162, 0), (249, 202, 3.14/2), 100)
    # path = finder.astar(a, (209, 162, 0), (172, 202, 3.14/2), 50)
    # path += finder.astar(a, (172, 202, 3.14/2), (249, 202, 3.14/2), 50)
    
    path = finder.astar(a, (209, 162, 0), (164, 242, 3.14/4), 50)
    path += finder.astar(a, (164, 242, 3.14/4), (249, 202, 3.14/2), 50)    
    
    cv2.imshow('Image', a)
    cv2.waitKey(0)    

    if type(path) is list:
        for p in path[::1]:
            draw_filled_tilted_rectangle(a, (p[0], p[1]), 17, 43, p[2])
            cv2.circle(a, [p[1], p[0]], 2, [255], -1)
            cv2.imshow('Image', a)
            cv2.waitKey(0)
    else:
        print(path)
    print(path)    
    cv2.imshow('Image', a)
    cv2.waitKey(0)
        
      
def main():  
    a = np.zeros((200, 200), np.uint8)

    cv2.circle(a, (100, 100), 50, 255, -1)
    cv2.circle(a, (180, 147), 10, 255, -1)
    
    finder = Astar((10, 15), 20, 0.3, 5)

    path = finder.astar(a, (16, 16, 0), (190, 190, -3.14/2))


    cv2.imshow('Image', a)
    cv2.waitKey(0)    

    if type(path) is list:
        for p in path:
            cv2.circle(a, [p[1], p[0]], 2, [255], -1)
    else:
        print(path)
    print(path)    
    cv2.imshow('Image', a)
    cv2.waitKey(0)

    # path = finder.get_path(gr, [15, 15, 0], [85, 85])
    # if type(path) is list:
    #     for i in range(len(path) - 1):
    #         p = path[i]
    #         p1 = path[i + 1]
    #         # cv2.circle(a, [p[1], p[0]], 2, [255], -1)
    #         cv2.line(a, (p[1], p[0]), (p1[1], p1[0]), [255] )
    # else:
    #     print(path)

    # print(path)
    
    # cv2.imshow('Image', a)
    # cv2.waitKey(0)


def test():
    a = np.zeros((200, 200), np.uint8)

    cv2.circle(a, (100, 100), 50, 255, -1)
    cv2.circle(a, (180, 147), 10, 255, -1)
    
    finder = Astar((10, 15), 20, 0.3, 5)

    path = finder.astar(a, (16, 16, 0), (190, 190, -3.14/2))


def coords_to_inds(coords, shape):
    row = int(shape[0] // 2 + coords[1])
    col = int(shape[1] // 2 + coords[0])
    return row, col    

c =[0, 0, 0], [206.453, -98.0267, 0.0], [24.3003, -31.9225, 0.0]

def inds_to_coords(inds, shape):
    y = float( - shape[0] // 2 + inds[0])
    x = float( - shape[1] // 2 + inds[1])
    return x, y


def json_data():
    import json
    import os
    os.chdir(os.path.dirname(__file__))
    s = json.load(open("mm.json"))
    a = np.array(s, np.uint8)
    a[np.where(a > 70)] = 255
    
    # a = np.flip(np.rot90(a), axis=1).astype(np.uint8).copy()
    
    cv2.imshow("",a)
    cv2.waitKey(0)
    
    finder = Astar((17, 45), 10, 0.3, 15)
    

    # path = finder.astar(a, (800, 800, 1.57), (594, 898, 1.57), 50, 10000)
    path = finder.astar(a, (400, 400, -0.0), (301, 606, 0.0), 50)
    if type(path) is list:
        for p in path[::1]:
            # draw_filled_tilted_rectangle(a, (p[0], p[1]), 17, 43, p[2])
            cv2.circle(a, [p[1], p[0]], 2, [255], -1)
            cv2.imshow('', a)
            cv2.waitKey(0)
    else:
        print(path)
    print(path)    
    cv2.imshow('', a)
    cv2.waitKey(0)    


if __name__ == "__main__":
    # parking()
    # main()
    json_data()
    import cProfile
    # cProfile.run("test()", sort="cumtime")