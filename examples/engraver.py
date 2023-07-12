import sys,os
sys.path.append('.')

from practistyle import Scene
import practistyle.data as data

scene = Scene(1024,768)

data_dir = data.path()
print(data_dir)
scene.load(os.path.join(data_dir,'scenes/engraver.json'))

def updating():
    import time
    while True:
        scene.update_for_tick(1/180.)
        time.sleep(1/180.)

from threading import Thread
t = Thread(target=updating)
t.start()