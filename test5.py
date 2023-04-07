import time
from digitaltwin import Scene, Workflow

projection = [[3507.176621132752,0.0,1218.55397352377],[0.0,3506.5191735910958,1038.1997907533078],[0.0,0.0,1.0]]
eye_to_hand_transform = [[0.03775400213484963,0.9992836506766881,-0.002611668637494993,0.18975821360828188],[0.9946931066717141,-0.03783062740787925,-0.09567897976933247,-0.8562761592934359],[-0.09570924126005752,0.0010144556158444772,-0.995408816525767,1.1566388127904739],[0,0,0,1]]

scene = Scene(1024, 768)
scene.load('./data/scenes/算法插件.json')
workflow = Workflow(scene)

scene.active_objs_by_name['camera'].set_calibration(projection,eye_to_hand_transform)

import pymeshlab as meshlab
ms = meshlab.MeshSet()
ms.load_new_mesh('/home/truman/Downloads/20230328094504929.ply')
m = ms.current_mesh()
vs = m.vertex_matrix()
fs = m.face_matrix()
vcs = m.vertex_color_matrix()[:,:3]

sample = int(len(vs) / 300000)
sample = 1 if sample < 1 else sample

scene.active_objs_by_name['camera'].draw_point_cloud(vs[::sample],vcs[::sample])

# scene.active_objs_by_name['camera'].clear_point_cloud()

workflow.start()


while True:
    scene.update_for_tick(1/180.)
    time.sleep(1/180.)
