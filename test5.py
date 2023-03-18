from time import time
from digitaltwin import Scene,Workflow
import pymeshlab as meshlab
import numpy as np


ms = meshlab.MeshSet()
ms.load_new_mesh("./data/models/lego.obj")
ms.generate_sampling_poisson_disk(radius=meshlab.Percentage(6))
ms.save_current_mesh('./data/models/lego.ply')

m = ms.current_mesh()
vs = m.vertex_matrix()
fs = m.face_matrix()
vcs = m.vertex_color_matrix()

scene = Scene(1024,768)
scene.load('./data/scenes/3D打印机.json')
workflow = Workflow(scene)

# scene.active_objs_by_name['camera'].draw_point_cloud('./data/models/lego.ply')
# scene.active_objs_by_name['camera'].clear_point_clound()

v = [vs[i] + [0,-0.5,0] for i in np.lexsort((vs[:,0],vs[:,1],vs[:,2]))]
scene.active_objs_by_name['robot'].signal_plan_move(None,v)

# workflow.start()

import time
while True:
    scene.update_for_tick(1/180.)
    time.sleep(1/180.)
