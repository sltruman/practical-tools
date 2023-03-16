from time import time
from digitaltwin import Scene,Workflow
import pymeshlab as meshlab
import numpy as np


ms = meshlab.MeshSet()
ms.load_new_mesh("./data/models/lego.obj")
ms.generate_sampling_poisson_disk(radius=meshlab.Percentage(4))
# ms.generate_sampling_volumetric()
# ms.generate_simplified_point_cloud(samplenum=6000)
ms.save_current_mesh('./data/models/lego.ply')


# point_cloud = open3d.io.read_point_cloud('./data/models/lego.ply')
# open3d.visualization.draw_geometries([point_cloud])


scene = Scene(1024,768)
scene.load('./data/scenes/3D打印机.json')
workflow = Workflow(scene)

scene.active_objs_by_name['camera'].draw_point_cloud('./data/models/lego.ply')
scene.active_objs_by_name['camera'].clear_point_clound()

# workflow.start()

import time
while True:
    scene.update_for_tick(1/180.)
    time.sleep(1/180.)
