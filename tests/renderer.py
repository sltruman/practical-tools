import numpy as np
import math as m
import matplotlib.pyplot as p
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation

origin = np.array([0,0,0])
cam = np.array([0,-1,0])
axis = np.array([0,0,1])

graph = p.figure(0)
view = graph.add_subplot(projection='3d')
p.show(block=False)
p.ion()

h = 0; v = 0
while True:
    h -= 1
    v -= 1

    if v < -89:
        v = 0
        h = 0

    print(h,v)

    r = Rotation.from_rotvec(np.array([0,0,1]) * h,True) * Rotation.from_rotvec(np.array([1,0,0]) * v,True)
    cam2 = r.apply(cam)

    cam2_norm = cam2 / np.linalg.norm(cam2)
    cosy = axis.dot(cam2_norm) / np.linalg.norm(cam2_norm) / np.linalg.norm(axis)
    
    view.cla()
    view.scatter([-2,2,-2,2],[-2,2,-2,2],[-2,-2,2,2])
    view.text(origin[0],origin[1],origin[2],f'{cosy}')
    view.plot([origin[0],cam2[0]],[origin[1],cam2[1]],[origin[2],cam2[2]])
    view.plot([origin[0],axis[0]],[origin[1],axis[1]],[origin[2],axis[2]])

    graph.canvas.draw()
    graph.canvas.flush_events()
    # input()

p.ioff()