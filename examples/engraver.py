import sys,os
sys.path.append('.')

from practistyle import Scene
import practistyle.data as data

scene = Scene(1024,768)

data_dir = data.path()
scene.load(os.path.join(data_dir,'scenes/engraver.json'))

def updating():
    import time
    try:
        while True:
            scene.update_for_tick(1/180.)
            time.sleep(1/180.)
    except:
        pass

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
        [0,0,0.00],
        [-0.11,0.07,0.00],
        [-0.11,-0.08,0.00],
        [0.11,0.07,0.00],
        [0.11,-0.08,0.00]
    ]:

    for rx,ry,rz in [
            [0,0,0],
            # [m.radians(8),0,0],
            # [0,m.radians(8),0],
            # [m.radians(-8),0,0],
            # [0,m.radians(-8),0]
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

# cv2.solvePnP(worldpoints,imgpoints,intrinsics,dist)

extrinsics = np.identity(4)
z_offset = [0,0,camera.focal + 0.00 + 0.000]
extrinsics[:3,:3] = Rotation.from_rotvec(rvecs[0][:,0]).as_matrix()
extrinsics[:3,3] = tvecs[0][:,0] + z_offset

print("标定外参",extrinsics) 
print("标定外参z补偿",z_offset)

total_error = 0
for i in range(len(worldpoints)):
    imgpoints2, _ = cv2.projectPoints(worldpoints[i], rvecs[i], tvecs[i], intrinsics, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    total_error += error

print("反投影误差: ", total_error/len(worldpoints))

from practistyle import Editor
editor = Editor(scene)
print('删除标定板')

editor.remove(checkerboard.name)

print('添加木板')
editor.add('ActiveObject','objects/board/board.urdf',[0,0,0.001],[0,0,0])

points_in_robot = []
T = np.array([-0.05,0.05,0.000])
editor.add('ActiveObject','objects/sphere/sphere.urdf',T,[0,0,0])
points_in_robot.append(T)
T = np.array([0.05,0.05,0.000])
editor.add('ActiveObject','objects/sphere/sphere.urdf',T,[0,0,0])
points_in_robot.append(T)
T = np.array([0.05,-0.05,0.000])
editor.add('ActiveObject','objects/sphere/sphere.urdf',T,[0,0,0])
points_in_robot.append(T)
T = np.array([-0.05,-0.05,0.000])
editor.add('ActiveObject','objects/sphere/sphere.urdf',T,[0,0,0])
points_in_robot.append(T)

camera.draw_fov()
rgba,depth = camera.rtt()
camera.draw_fov(False)
img = np.frombuffer(rgba,np.uint8).reshape((camera.pixels_h,camera.pixels_w,4))

img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

p1 = [429,284]
p2 = [611,301]
p3 = [593,483]
p4 = [411,465]

points_in_camera = []

for x,y in [p1,p2,p3,p4]:
    z = extrinsics[2,3]
    p = np.linalg.inv(intrinsics) @ [x,y,1] * z
    R = Rotation.from_matrix(extrinsics[:3,:3])
    T = extrinsics[:3,3]

    axes = [
        [1,0,0],
        [0,1,0],
        [0,0,-1]
    ]
        
    p = p @ axes

    print('相机坐标系',p)
    points_in_camera.append(p)

center_in_camera = np.array([0.0,0.0,0.0])
for p in points_in_camera:
    center_in_camera += p
center_in_camera /= len(points_in_camera)
print(center_in_camera)

center_in_robot = np.array([0.0,0.0,0.0])
for p in points_in_robot:
    center_in_robot += p
    
center_in_robot /= len(points_in_robot)
print(center_in_robot)

    
t = center_in_robot - center_in_camera
r = 0
r_axis = np.array([0.,0.,0.])
for i in range(4):
    point_direction1 = points_in_robot[i] - center_in_robot
    point_direction2 = points_in_camera[i] - center_in_camera
    point_direction1_norm = np.linalg.norm(point_direction1)
    point_direction2_norm = np.linalg.norm(point_direction2)
    point_direction1 = point_direction1 / np.linalg.norm(point_direction1)
    point_direction2 = point_direction2 / np.linalg.norm(point_direction2)
    r += np.dot(point_direction1,point_direction2) / point_direction1_norm * point_direction2_norm
    r_axis += np.cross(point_direction2,point_direction1)

r /= 4
r_axis /= 4
r_axis /= np.linalg.norm(r_axis)

print('手眼标定',r_axis,m.degrees(r),t.tolist())
r = R.from_rotvec(r_axis * m.degrees(r))

for p_t in points_in_camera:
    p_t = r.apply(p_t) + t
    editor.add('ActiveObject','objects/sphere/sphere.urdf',p_t,[0,0,0])
    print('机械臂坐标系',p_t)

cv2.imshow(f'click', img)
cv2.waitKey()
