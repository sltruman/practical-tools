import time
from digitaltwin import Scene, Workflow

projection = [[3507.176621132752,0.0,1218.55397352377],[0.0,3506.5191735910958,1038.1997907533078],[0.0,0.0,1.0]]
eye_to_hand_transform = [
    [-0.9955322430611627,0.0028817025222408156,0.09437822214998369,-0.24240494262053444],
    [0.0013186702509287958,0.9998610162318016,-0.016619546584627724,-0.4114167017006515],
    [-0.09441299769834981,-0.016420840736168302,-0.9953976802540438,1.2696347107418071],
    [0.0,0.0,0.0,1.0]]

scene = Scene(1024, 768)
scene.load('./digitaltwin_data/scenes/算法插件.json')
workflow = Workflow(scene)

scene.active_objs_by_name['camera'].set_calibration(projection,eye_to_hand_transform)

# import pymeshlab as meshlab
# ms = meshlab.MeshSet()
# ms.load_new_mesh('/home/truman/Downloads/20230401135657267/Builder/foreground/output/20230401135657267.ply')
# m = ms.current_mesh()
# vs = m.vertex_matrix()
# fs = m.face_matrix()
# vcs = m.vertex_color_matrix()[:,:3]
# scene.active_objs_by_name['camera'].draw_point_cloud(vs,vcs)

# scene.active_objs_by_name['camera'].clear_point_cloud()

workflow.start()

while True:
    scene.update_for_tick(1/180.)
    time.sleep(1/180.)
