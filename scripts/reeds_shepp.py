#!/bin/python

import numpy as np
import math


# parameters initiation
max_c = 0.006  # max curvature

from curves import *


# class for PATH element


def calc_curvature(x, y, yaw, directions):
    c, ds = [], []

    for i in range(1, len(x) - 1):
        dxn = x[i] - x[i - 1]
        dxp = x[i + 1] - x[i]
        dyn = y[i] - y[i - 1]
        dyp = y[i + 1] - y[i]
        dn = math.hypot(dxn, dyn)
        dp = math.hypot(dxp, dyp)
        dx = 1.0 / (dn + dp) * (dp / dn * dxn + dn / dp * dxp)
        ddx = 2.0 / (dn + dp) * (dxp / dp - dxn / dn)
        dy = 1.0 / (dn + dp) * (dp / dn * dyn + dn / dp * dyp)
        ddy = 2.0 / (dn + dp) * (dyp / dp - dyn / dn)
        curvature = (ddy * dx - ddx * dy) / (dx ** 2 + dy ** 2)
        d = (dn + dp) / 2.0

        if np.isnan(curvature):
            curvature = 0.0

        if directions[i] <= 0.0:
            curvature = -curvature

        if len(c) == 0:
            ds.append(d)
            c.append(curvature)

        ds.append(d)
        c.append(curvature)

    ds.append(ds[-1])
    c.append(c[-1])

    return c, ds


def check_path(sx, sy, syaw, gx, gy, gyaw, maxc):
    paths = calc_all_paths(sx, sy, syaw, gx, gy, gyaw, maxc)

    assert len(paths) >= 1

    for path in paths:
        assert abs(path.x[0] - sx) <= 0.01
        assert abs(path.y[0] - sy) <= 0.01
        assert abs(path.yaw[0] - syaw) <= 0.01
        assert abs(path.x[-1] - gx) <= 0.01
        assert abs(path.y[-1] - gy) <= 0.01
        assert abs(path.yaw[-1] - gyaw) <= 0.01

        # course distance check
        d = [math.hypot(dx, dy)
             for dx, dy in zip(np.diff(path.x[0:len(path.x) - 1]),
                               np.diff(path.y[0:len(path.y) - 1]))]

        for i in range(len(d)):
            assert abs(d[i] - STEP_SIZE) <= 0.001


def main():
    import numpy as np
    import matplotlib.pyplot as plt

    sx, sy, syaw = 66, 4, -1.2  # Начальные координаты и угол
    gx, gy, gyaw = 90, 90, -1.57  # Конечные координаты и угол
    max_c = 0.1  # Максимальная кривизна

    # Находим оптимальный путь
    optimal_path = calc_optimal_path(*(162, 209, 0), *(202, 157, 3.14/2), 0.1, 5)


    # Визуализация
    plt.figure(figsize=(10, 10))
    plt.plot(optimal_path.x, optimal_path.y, label='Оптимальный путь', marker='o')
    plt.quiver(optimal_path.x, optimal_path.y, 
            np.cos(optimal_path.yaw), np.sin(optimal_path.yaw), 
            scale=5, color='r', label='Направление движения')
    plt.scatter(sx, sy, color='g', label='Начальная позиция')
    plt.scatter(gx, gy, color='b', label='Конечная позиция')
    plt.title('Визуализация пути')
    plt.xlabel('X (м)')
    plt.ylabel('Y (м)')
    plt.axis('equal')
    plt.legend()
    plt.grid()
    plt.show()


def test():
    sx, sy, syaw = 4, 66, -1.2  # Начальные координаты и угол
    gx, gy, gyaw = 90, 90, -1.57  # Конечные координаты и угол
    max_c = 0.01

    optimal_path = calc_optimal_path(sx, sy, syaw, gx, gy, gyaw, max_c, 1)

            
if __name__ == "__main__":
    main()
    import cProfile
    
    # cProfile.run("test()")
    pass


# print()     