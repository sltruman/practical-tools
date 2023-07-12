import sys,os
sys.path.append('.')

from practistyle import Scene,Workflow
from practistyle import data

scene = Scene(1024,768)
workflow = Workflow(scene)

data_dir = data.path()
scene.load(os.path.join(data_dir,'scenes/calibration_circled.json'))

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
import math as m 

def single_calibrate(path,image_nums=16,pattern_size = (7, 11),image_size=(1024, 768),resolution=40):
    image_pts_list = []
    obj_pts_list = []

    obj_pts = []

    image_list = []
    # for row in range(pattern_size[1]):
    #     for col in range(pattern_size[0]):
    #         obj_pts.append([row * resolution, col * resolution, 0])

    for row in range(pattern_size[1]):
        for col in range(pattern_size[0]):
            if row % 2 == 0:
                obj_pts.append([row * resolution * 0.5, col * resolution, 0])
            else:
                obj_pts.append([row * resolution * 0.5, col * resolution + 0.5 * resolution, 0])

    for i in range(image_nums):
        img_path = f'{path}/{i}.png'
        print('img: ', img_path)
        img = cv2.imread(img_path, 0)

        cv2.equalizeHist(img, img)
        # is_found, corners_nx1x2 = cv2.findChessboardCorners(img, patternSize=pattern_size)
        # 对图片取负片
        for row in range(img.shape[0]):
            for col in range(img.shape[1]):
                img[row][col] = 255 - img[row][col]


        is_found, corners_nx1x2 = cv2.findCirclesGrid(img, patternSize=pattern_size, flags=cv2.CALIB_CB_ASYMMETRIC_GRID)
        if not is_found:
            print("not find!")
        cv2.drawChessboardCorners(img, pattern_size, corners_nx1x2, is_found)
        cv2.namedWindow('img', 0)
        cv2.imshow('img', img)
        cv2.waitKey(30)

        if is_found:
            image_pts_list.append(np.float32(corners_nx1x2).reshape(-1, 2))
            obj_pts_list.append(np.float32(obj_pts))
            image_list.append(i)

    (
        re_projection_err,
        camera_matrix,
        dist_coeffs,
        rvecs,
        tvecs,
        _,
        _,
        re_projection_errs,
    ) = cv2.calibrateCameraExtended(
        objectPoints=obj_pts_list,
        imagePoints=image_pts_list,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None,
        # flags=cv2.CALIB_RATIONAL_MODEL
    )

    print('err: ', re_projection_err,camera_matrix)
    return camera_matrix, dist_coeffs, image_pts_list, obj_pts_list, image_list


i=0
for x,y,z,rx,ry,rz in [
    [0,0,0.08,m.radians(25),0,0],
    [0,0,0.08,0,m.radians(25),0],
    [0,0,0.08,m.radians(-25),0,0],
    [0,0,0.08,0,m.radians(-25),0],

    [-0.11,0.07,0.08,m.radians(25),0,0],
    [-0.11,0.07,0.08,0,m.radians(25),0],
    [-0.11,0.07,0.08,m.radians(-25),0,0],
    [-0.11,0.07,0.08,0,m.radians(-25),0],

    [-0.11,-0.07,0.08,m.radians(25),0,0],
    [-0.11,-0.07,0.08,0,m.radians(25),0],
    [-0.11,-0.07,0.08,m.radians(-25),0,0],
    [-0.11,-0.07,0.08,0,m.radians(-25),0],

    [0.11,0.07,0.08,m.radians(25),0,0],
    [0.11,0.07,0.08,0,m.radians(25),0],
    [0.11,0.07,0.08,m.radians(-25),0,0],
    [0.11,0.07,0.08,0,m.radians(-25),0],
 
    [0.11,-0.07,0.08,m.radians(25),0,0],
    [0.11,-0.07,0.08,0,m.radians(25),0],
    [0.11,-0.07,0.08,m.radians(-25),0,0],
    [0.11,-0.07,0.08,0,m.radians(-25),0],
    ]:
        
    checkerboard.set_pos([x,y,z])
    checkerboard.set_rot([rx,ry,rz])
    camera.draw_fov()
    rgba,depth = camera.rtt()
    camera.draw_fov(False)
    img = np.frombuffer(rgba,np.uint8).reshape((camera.pixels_h,camera.pixels_w,4))
    img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    cv2.imwrite(f'images/{i}.png', img)

    i+=1


np.set_printoptions(suppress=True)

single_calibrate('images')