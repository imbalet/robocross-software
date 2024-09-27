import dubins
import cv2
import numpy as np

q0 = (200, 200, 0)
q1 = (200, 200, 3.14)
turning_radius = 50.0
step_size = 5

path = dubins.shortest_path(q0, q1, turning_radius)
p, _ = path.sample_many(step_size)

a = np.zeros((400, 400), np.uint8)

for i in range(len(p)):
    p[i] = (int(p[i][0]), int(p[i][1]), p[i][2])

for i in range(len(p) - 1):
    cv2.line(a, p[i][:2], p[i + 1][:2], 127, 1)

cv2.imshow('', a)
cv2.waitKey(0)

print()