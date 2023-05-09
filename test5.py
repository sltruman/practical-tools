from time import time
from digitaltwin import Scene,Workflow,Render
from threading import Thread

scene = Scene(1024,768)
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
scene.active_objs_by_name['camera'].draw_point_cloud_from_depth_pixels(depth,rgba,1024,768)