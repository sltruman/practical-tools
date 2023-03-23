import time
from digitaltwin import Scene, Workflow

projection = [
    [2063.6201171875,0.0,966.2369995117188],
    [0.0,2063.840087890625,590.551025390625],
    [0.0,0.0,1.0]]

eye_to_hand_transform = [
    [-0.03848572352572521,-0.9986251185481502,-0.035591033793917766,0.9516957587752023],
    [-0.9992461560238449,0.03827916479278677,0.006467241624670584,-0.900332425706558],
    [-0.005095954886352403,0.03581310018049166,-0.9993455123725961,0.953975140167797],
    [0.0,0.0,0.0,1.0]]


scene = Scene(1024, 768)
scene.load('./data/scenes/点云.json')
workflow = Workflow(scene)

scene.active_objs_by_name['camera'].set_calibration(projection,eye_to_hand_transform)
scene.active_objs_by_name['camera'].draw_point_cloud('./data/models/test.ply')
scene.active_objs_by_name['camera'].draw_point_cloud('./data/models/3.ply')
scene.active_objs_by_name['camera'].draw_point_cloud('./data/models/2.ply')
# scene.active_objs_by_name['camera'].clear_point_cloud()

# workflow.start()

while True:
    scene.update_for_tick(1/180.)
    time.sleep(1/180.)
