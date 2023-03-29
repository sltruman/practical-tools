import time
from digitaltwin import Scene, Workflow

projection = [[3507.176621132752,0.0,1218.55397352377],[0.0,3506.5191735910958,1038.1997907533078],[0.0,0.0,1.0]]

eye_to_hand_transform = [[-0.03848572352572521,-0.9986251185481502,-0.035591033793917766,0.9516957587752023],[-0.9992461560238449,0.03827916479278677,0.006467241624670584,-0.900332425706558],[-0.005095954886352403,0.03581310018049166,-0.9993455123725961,1.853975140167797],[0.0,0.0,0.0,1.0]]

scene = Scene(1024, 768)
scene.load('./data/scenes/算法插件.json')
workflow = Workflow(scene)

scene.active_objs_by_name['camera2'].set_calibration(projection,eye_to_hand_transform)
scene.active_objs_by_name['camera2'].draw_point_cloud('/home/truman/Downloads/20230328191753403.ply')

# scene.active_objs_by_name['camera'].clear_point_cloud()

#workflow.start()

while True:
    scene.update_for_tick(1/180.)
    time.sleep(1/180.)
