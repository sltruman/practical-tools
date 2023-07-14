import sys,os
sys.path.append('.')

from practistyle import Scene
import practistyle.data as data

scene = Scene(1024,768)

data_dir = data.path()
scene.load(os.path.join(data_dir,'scenes/engraver.json'))

def updating():
    import time
    while True:
        scene.update_for_tick(1/180.)
        time.sleep(1/180.)

from threading import Thread
t = Thread(target=updating)
t.start()

camera = scene.active_objs_by_name['camera']
checkerboard = scene.active_objs_by_name['checkerboard']

import cv2
import numpy as np
import math as m
from scipy.spatial.transform import Rotation

def find_corners(gray, chess_col, chess_row):
    ret, corners = cv2.findChessboardCorners(gray, (chess_col, chess_row), None)

    if not ret:
        print('没有角点被找到！')
        return ret, corners

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)

    return ret, corners

distance = 0.030
c=9; r=6
worldcorners = np.zeros((c * r, 3), np.float32)
worldcorners[:, :2] = np.mgrid[0:c,0:r].T.reshape(-1, 2) * distance

worldpoints = []
imgpoints = []

for x,y,z in [
        [0,0,0.02],
        [-0.11,0.07,0.02],
        [-0.11,-0.08,0.02],
        [0.11,0.07,0.02],
        [0.11,-0.08,0.02]
    ]:

    for rx,ry,rz in [
            [0,0,0],
            [m.radians(8),0,0],
            [0,m.radians(8),0],
            [m.radians(-8),0,0],
            [0,m.radians(-8),0]
        ]:

        i = len(imgpoints)
        print(i)
        
        checkerboard.set_pos([x,y,z])
        checkerboard.set_rot([rx,ry,rz])
        camera.draw_fov()
        rgba,depth = camera.rtt()
        camera.draw_fov(False)
        img = np.frombuffer(rgba,np.uint8).reshape((camera.pixels_h,camera.pixels_w,4))
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        cv2.imwrite(f'images/{i}.png', img)
        ret,corners = find_corners(gray,c,r)
        if not ret: continue

        cv2.drawChessboardCorners(img, (c, r), corners, ret)

        imgpoints.append(corners)
        worldpoints.append(worldcorners)

np.set_printoptions(suppress=True)

print('相机内参',camera.get_intrinsics())
print('相机外参',camera.pos,camera.rot)

ret, intrinsics, dist, rvecs, tvecs = cv2.calibrateCamera(worldpoints, imgpoints, gray.shape[::-1], None, None)
print("标定内参",intrinsics.tolist())

extrinsics = np.identity(4)
z_offset = [0,0,camera.focal + 0.02 + 0.005]
extrinsics[:3,:3] = Rotation.from_rotvec(rvecs[0][:,0]).as_matrix()
extrinsics[:3,3] = tvecs[0][:,0] + z_offset

print("标定外参",extrinsics) 
print("标定外参z补偿",z_offset)

total_error = 0
for i in range(len(worldpoints)):
    imgpoints2, _ = cv2.projectPoints(worldpoints[i], rvecs[i], tvecs[i], intrinsics, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    total_error += error

print("反投影误差: ", total_error/len(worldpoints))

from practistyle import Editor
editor = Editor(scene)
print('删除标定板')

editor.remove(checkerboard.name)

print('添加木板')
editor.add('ActiveObject','objects/board/board.urdf',[0,0,0.001],[0,0,0])

points_in_robot = []
T = np.identity(4)
T[:3,3] = [-0.05,0.05,0.002]
editor.add('ActiveObject','objects/sphere/sphere.urdf',T[:3,3],[0,0,0])
points_in_robot.append(T[:3,3])
T[:3,3] = [0.05,0.05,0.005]
editor.add('ActiveObject','objects/sphere/sphere.urdf',T[:3,3],[0,0,0])
points_in_robot.append(T[:3,3])
T[:3,3] = [0.05,-0.05,0.002]
editor.add('ActiveObject','objects/sphere/sphere.urdf',T[:3,3],[0,0,0])
points_in_robot.append(T[:3,3])
T[:3,3] = [-0.05,-0.05,0.002]
editor.add('ActiveObject','objects/sphere/sphere.urdf',T[:3,3],[0,0,0])
points_in_robot.append(T[:3,3])

camera.draw_fov()
rgba,depth = camera.rtt()
camera.draw_fov(False)
img = np.frombuffer(rgba,np.uint8).reshape((camera.pixels_h,camera.pixels_w,4))
img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


p1 = [420,292]
p2 = [603,292]
p3 = [603,475]
p4 = [420,475]

points_in_camera = []

for x,y in [p1,p2,p3,p4]:
    z = extrinsics[2,3]
    p = np.linalg.inv(intrinsics) @ [x,y,1] * z
    print('相机坐标系',p)
    points_in_camera.append(p)

r,t = cv2.calibrateHandEye([np.identity(3)] * 4,points_in_robot,[np.identity(3)]*4,points_in_camera)
print('手眼标定',r.tolist(),t[:,0])

for p_t in points_in_camera:
    print('机械臂坐标系',Rotation.from_matrix(r).apply(p_t) + t[:,0])
    


cv2.imshow(f'click', img)
while True:
    cv2.waitKey(100)
