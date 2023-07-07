import sys,os
sys.path.append('.')

from practistyle import Scene
import practistyle.data as data

scene = Scene(1024,768)

data_dir = data.path()
print(data_dir)
scene.load(os.path.join(data_dir,'scenes/2d_printer.json'))

def updating():
    import time
    while True:
        scene.update_for_tick(1/180.)
        time.sleep(1/180.)

from threading import Thread
t = Thread(target=updating)
t.start()

rgba,depth = scene.active_objs_by_name['camera'].rtt()

import cv2
import numpy as np
 
img = np.frombuffer(rgba,dtype=np.uint8).reshape((768,1024,4))
cv2.imwrite("Haha.png", cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA))