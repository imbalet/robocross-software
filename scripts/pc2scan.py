import numpy as np
import ros2_numpy as rnp

from rclpy.node import Node
from utils import decart_to_polar
from sensor_msgs.msg import PointCloud2, LaserScan

x_max = 20.0
x_min = 0.7
y_max = 100.
y_min = -100.
z_max = 0.0
z_min = -0.3
angle_max = 0.7
angle_min = -0.7
rays_number = 300
frame_id = 'front_camera_link'
angleIncrement = (abs(angle_min) + abs(angle_max)) / rays_number


def convert_from_msg(msg):
    scanData = np.zeros([rays_number, 1], np.float32)
    
    data = rnp.numpify(msg)
    data = np.array(data['xyz'], dtype=np.float32)
    condition = (data[:, 2] < z_max) & (data[:, 2] > z_min) & (data[:, 1] < y_max) & \
                (data[:, 1] > y_min) & (data[:, 0] < x_max) & (data[:, 0] > x_min)
    data = data[condition]
    lim = scanData.shape[0]
    for x, y, z in data:
        rad, a = decart_to_polar(x, y)
        i = int((a - angle_min) // angleIncrement + 1)
        if i < 0 or i > lim: continue
        if scanData[i] == 0:
            scanData[i] = rad
    
    new_msg = LaserScan()
    new_msg.header.stamp = msg.header.stamp
    new_msg.header.frame_id = frame_id
    new_msg.angle_min = angle_min
    new_msg.angle_max = angle_max
    new_msg.angle_increment = angleIncrement
    new_msg.range_min = x_min
    new_msg.range_max = x_max
    new_msg.ranges = list(map(float, scanData))
    
    return new_msg
    
