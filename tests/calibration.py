import cv2
import sys,os
sys.path.append('.')

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


distance = 0.028
c=8; r=5
worldcorners = np.zeros((c * r, 3), np.float32)
worldcorners[:, :2] = np.mgrid[0:c,0:r].T.reshape(-1, 2) * distance

worldpoints = []
imgpoints = []

for i in range(9):
        print(i)
        
        img = cv2.imread(f'C:/Users/SLTru/Pictures/Camera Roll/{i}.jpg')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        ret,corners = find_corners(gray,c,r)
        if not ret: continue

        cv2.drawChessboardCorners(img, (c, r), corners, ret)


        imgpoints.append(corners)
        worldpoints.append(worldcorners)

np.set_printoptions(suppress=True)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(worldpoints, imgpoints, gray.shape[::-1], None, None)
print("内参",mtx)

extrinsics = np.identity(4)
z_offset = [0,0,0]
extrinsics[:3,:3] = Rotation.from_rotvec(rvecs[4][:,0]).as_matrix()
extrinsics[:3,3] = tvecs[0][:,0] + z_offset
print('外参',extrinsics[:3,3])

# 反投影误差
total_error = 0
for i in range(len(worldpoints)):
    imgpoints2, _ = cv2.projectPoints(worldpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    total_error += error
print(("total error: "), total_error/len(worldpoints))

for i in range(9):
    print(i)
    img = cv2.imread(f'C:/Users/SLTru/Pictures/Camera Roll/{i}.jpg')
    dst = cv2.undistort(img, mtx, dist)
    cv2.imwrite(f'C:/Users/SLTru/Pictures/Camera Roll/{i}.png',dst)

points_in_robot = []
T = np.array([150,300,0]) / 1000
points_in_robot.append(T) 
T = np.array([300,300,0]) / 1000
points_in_robot.append(T) 
T = np.array([300,150,0]) / 1000
points_in_robot.append(T) 
T = np.array([150,150,0]) / 1000
points_in_robot.append(T) 

img = cv2.imread(f'C:/Users/SLTru/Pictures/Camera Roll/9.jpg')
dst = cv2.undistort(img, mtx, dist)
# mtx2, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(1024,768),0) # 自由比例参数
# print('新内参',mtx2)
intrinsics = mtx

# x,y,w,h = roi
# print(roi)
# dst = dst[y:y+h, x:x+w]
# dst = cv2.resize(dst,(w,h))

cv2.imwrite(f'C:/Users/SLTru/Pictures/Camera Roll/9.png',dst)
cv2.imshow('',dst)

p1 = [442,277]
p2 = [662,279]
p3 = [654,491]
p4 = [441,485]

points_in_camera = []
for x,y in [p1,p2,p3,p4]:
    z = 0.30 #extrinsics[2,3]
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
    r += np.dot(point_direction1,point_direction2) / point_direction1_norm / point_direction2_norm
    r_axis += np.cross(point_direction2,point_direction1)

r /= 4
r_axis /= 4
r_axis /= np.linalg.norm(r_axis)

print('手眼标定',r_axis,m.degrees(r),t.tolist())
r = R.from_rotvec(r_axis * m.degrees(r))

for p_t in points_in_camera:
    p_t = r.apply(p_t) + t
    print('机械臂坐标系',p_t)

cv2.waitKey()