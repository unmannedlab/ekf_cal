import cv2
import numpy as np

focal = 10
width  = 640
height = 480

points_lidar = [
    [0,0,4.5],
    [10,0,4.5],
    [0,10,4.5],
    [-10,0,4.5],
    [0,-10,4.5],
    [1.88,-0.42,4.50],
    [1.85,-0.42,4.49],
    [1.84,-0.42,4.49],
    [1.83,-0.42,4.51],
    [1.82,-0.42,4.52],
    [1.81,-0.41,4.52]]

r_vec = cv2.Rodrigues(np.array([0, 0, 0], dtype=float))
t_vec = np.array([0, 0, 0], dtype=float)
d_vec = np.array([0, 0, 0, 0], dtype=float)

camera_mat = np.array([
        [focal,0,width/2],
        [0,focal,height/2],
        [0,0,1]], dtype=float)

for points in points_lidar:
    out, jac = cv2.projectPoints(np.array(points, dtype=float), (0,0,0), (0,0,0), camera_mat, d_vec)
    uv = out[0][0]
    if uv[0] < 0 or uv[1] < 0 or uv[0] > width or uv[1] > height:
        continue
    print(uv)