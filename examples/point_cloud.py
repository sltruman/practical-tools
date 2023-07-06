import os
from digitaltwin import Scene, Workflow,Editor
import digitaltwin_data

projection = [[
                2393.230224609375,
                0.0,
                951.794189453125
            ],[
                0.0,
                2393.364501953125,
                558.6798095703125
            ],[
                0.0,
                0.0,
                1.0
            ]]

eye_to_hand_transform = [
                    [
                        0.08766222494286922,
                        0.9954482545931286,
                        0.037391265631955106,
                        0.7793440568869567
                    ],
                    [
                        0.9619166950744155,
                        -0.09434572446817567,
                        0.25654464720919273,
                        -0.2510889858020945
                    ],
                    [
                        0.258904627334428,
                        0.013478008089797142,
                        -0.9658088512965427,
                        1.2629839393068865
                    ],
                    [
                        0.0,
                        0.0,
                        0.0,
                        1.0
                    ]
                ]

scene = Scene(1024, 768)
editor = Editor(scene)

data_dir = digitaltwin_data.get_data_path()
scene.load(os.path.join(data_dir,'scenes/算法插件.json'))

workflow = Workflow(scene)

# print(scene.active_objs_by_name['robot'].get_joints())

scene.active_objs_by_name['camera'].set_calibration(projection,eye_to_hand_transform)
editor.add_cube([0,0,-0.7],[0,0,0],[0.5,0.5,0.7])

import pymeshlab as meshlab
ms = meshlab.MeshSet()
ms.load_new_mesh('/home/truman/Downloads/20230530194453412/Builder/foreground/output/20230530194453412.ply')
m = ms.current_mesh()
vs = m.vertex_matrix()
fs = m.face_matrix()
vcs = m.vertex_color_matrix()[:,:3]

scene.active_objs_by_name['camera'].draw_point_cloud(vs,vcs)
# scene.active_objs_by_name['camera'].clear_point_cloud()

workflow.start()

while True:
    import time
    scene.update_for_tick(1/180.)
    time.sleep(1/180.)
