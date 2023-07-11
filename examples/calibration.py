import sys,os
sys.path.append('.')

from practistyle import Scene,Workflow
from practistyle import data

scene = Scene(1024,768)
workflow = Workflow(scene)

data_dir = data.path()
scene.load(os.path.join(data_dir,'scenes/calibration.json'))

def updating():
    import time
    while True:
        scene.update_for_tick(1/180.)
        time.sleep(1/180.)

from threading import Thread
t = Thread(target=updating)
t.start()

camerareal = scene.active_objs_by_name['camerareal']
camera = scene.active_objs_by_name['camera']
checkerboard = scene.active_objs_by_name['checkerboard']

import cv2
import numpy as np
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

for x,y,z,rx,ry,rz in [
    [0,0,0,0,0,0],
    [0,0,0,0,0,1.57*0.25],
    [0,0,0,0,0,1.57*0.5],
    [0,0,0,0,0,1.57*0.75],

    [-0.125/2,0.1/2,0,0,0,0],
    [-0.125/2,0.1/2,0,0,0,0.785],
    [-0.125/2,0.1/2,0,0,0,-0.785],

    [-0.125/2,-0.1/2,0,0,0,0],
    [-0.125/2,-0.1/2,0,0,0,0.785],
    [-0.125/2,-0.1/2,0,0,0,-0.785],

    [0.125/2,0.1/2,0,0,0,0],
    [0.125/2,0.1/2,0,0,0,0.785],
    [0.125/2,0.1/2,0,0,0,-0.785],
 
    [0.125/2,-0.1/2,0,0,0,0],
    [0.125/2,-0.1/2,0,0,0,0.785],
    [0.125/2,-0.1/2,0,0,0,-0.785],
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
        
        ret,corners = find_corners(gray,c,r)
        if not ret: continue

        cv2.drawChessboardCorners(img, (c, r), corners, ret)
        cv2.imshow('FoundCorners', img)
        cv2.waitKey(500)  
        cv2.imwrite(f'images/{i}.png', img)

        imgpoints.append(corners)
        worldpoints.append(worldcorners)

np.set_printoptions(suppress=True)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(worldpoints, imgpoints, gray.shape[::-1], None, None)
print("内参",mtx)

# 反投影误差
total_error = 0
for i in range(len(worldpoints)):
    imgpoints2, _ = cv2.projectPoints(worldpoints[i], rvecs[i], tvecs[i], mtx, dist)
    print(tvecs[i]/2)
    error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    total_error += error
print(("total error: "), total_error/len(worldpoints))

print("内参",mtx)
print('真内参',camera.get_intrinsics())
camerareal.set_intrinsics(mtx)
camerareal.draw_point_cloud_from_depth_pixels(depth,rgba,1024,768)
