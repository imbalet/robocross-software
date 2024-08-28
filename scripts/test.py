import numpy as np
import cv2
import heapq

class Node:
    def __init__(self, position, cost, heuristic, parent=None):
        self.position = position
        self.cost = cost
        self.heuristic = heuristic
        self.parent = parent

    def f(self):
        return self.cost + self.heuristic

    def __lt__(self, other):
        return self.f() < other.f()

def heuristic(a, b):
    # return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Манхэттенское расстояние
    return ((a[0] - b[0])**2 + (a[1] - b[1]) ** 2)**0.5

def astar(matrix, start, goal):
    if matrix[start[0], start[1]] >= 127 or matrix[goal[0], goal[1]] >= 127:
        return None  # Начальная или конечная клетка занята

    open_list = []
    closed_list = set()

    start_node = Node(start, 0, heuristic(start, goal))
    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # Путь в правильном порядке

        # Определение соседних клеток (вверх, вниз, влево, вправо)
        neighbors = [
            (1, -1),   # Вправо
            (-1, -1),  # Влево
            
            (1, 1),   # Вправо
            (-1, 1),  # Влево
            
            (1, 0),   # Вниз
            (-1, 0),  # Вверх
        ]

        for dx, dy in neighbors:
            neighbor_position = (current_node.position[0] + dx, current_node.position[1] + dy)

            # Проверяем, что сосед находится в пределах матрицы
            if (0 <= neighbor_position[0] < matrix.shape[0] and
                    0 <= neighbor_position[1] < matrix.shape[1]):
                cell_cost = matrix[neighbor_position]

                # Пропускаем клетки, которые заняты
                if cell_cost >= 127 or neighbor_position in closed_list:
                    continue

                new_cost = current_node.cost + cell_cost

                for open_node in open_list:
                    if open_node.position == neighbor_position and new_cost >= open_node.cost:
                        break
                else:
                    neighbor_node = Node(neighbor_position, new_cost, heuristic(neighbor_position, goal), current_node)
                    heapq.heappush(open_list, neighbor_node)

    return None  # Если путь не найден

# Пример использования
if __name__ == "__main__":
    a = np.zeros((100, 100), dtype=np.uint8)
    cv2.circle(a, [50, 50], 40, [127], -1)  # Занятая область

    start = (15, 15)  # Начальная позиция
    end = (85, 85)    # Конечная позиция

    path = astar(a, start, end)

    if isinstance(path, list):
        for p in path:
            cv2.circle(a, [p[1], p[0]], 1, [255], -1)  # Изменяем порядок координат для OpenCV
            # a[p[1]] [p[0]] = 255
    else:
        print("Путь не найден")

    cv2.imshow('Image', a)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
