from time import time
from threading import Thread
import cv2
import numpy as np
import math

from digitaltwin import Scene,Workflow,Editor

scene = Scene(1024,768)
editor = Editor(scene)
workflow = Workflow(scene)

scene.load('./data/scenes/标定测试.json')

def updating():
    import time
    while True:
        scene.update_for_tick(1/180.)
        time.sleep(1/180.)

t = Thread(target=updating)
t.start()

rgba,depth = scene.active_objs_by_name['camera'].rtt()
intrinsics = scene.active_objs_by_name['camera'].get_intrinsics()
extrinsics = scene.active_objs_by_name['camera'].get_extrinsics()

properties = editor.add('Camera3DReal',"./cameras/camera3d/camera3d.urdf",[-0.032747,0.564563,1.1767],[0,0,0],[0,0,0])

scene.active_objs_by_name['camera3dreal'].set_intrinsics(intrinsics)
scene.active_objs_by_name['camera3dreal'].set_extrinsics(extrinsics)

scene.active_objs_by_name['camera3dreal'].draw_point_cloud_from_depth_pixels(depth,rgba,1024,768)

img = np.frombuffer(rgba, dtype=np.uint8).reshape((768, 1024, 4))
cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA,img)
cv2.imwrite('rgba.png',img)

depth = np.frombuffer(depth, dtype=np.float32).reshape((768, 1024, 1))
cv2.imwrite('dep.tiff',depth)

min,max,_,_ = cv2.minMaxLoc(depth)
gray = (depth * (255 / max)).astype(np.uint8)
cv2.imwrite('gray.png',gray)


