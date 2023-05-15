import time
from digitaltwin import Scene,Editor, Workflow

scene = Scene(1024, 768,'/home/truman/Desktop/digital-twin','/home/truman/Desktop/digital-twin/')
scene.load('./scenes/空.json')
editor = Editor(scene)
workflow = Workflow(scene)

editor.add('ActiveObject','objects/box/box.urdf',[0,0,0],[0,0,0],[1,0,0])
editor.add('ActiveObject','objects/cylinder/cylinder.urdf',[0,1,0],[0,0,0],[1,0,0])
obj = editor.add('Robot','robots/franka_panda/franka_panda.urdf',[2,0,0],[0,0,0],[1,0,0])

name = obj['name']
scene.active_objs_by_name[name].set_end_effector('end_effectors/gripper2/gripper2.urdf')

print(scene.active_objs_by_name[name].get_joints())

while True:
    scene.update_for_tick(1/180.)
    time.sleep(1/180.)
