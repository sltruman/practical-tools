import time
import os
import matplotlib.pyplot as plt
from practistyle import Scene,Renderer,Workflow
import practistyle.data as data

scene = Scene(1024,768)
workflow = Workflow(scene)

data_dir = data.path()
scene.load(os.path.join(data_dir,'scenes/深度图.json'))

workflow.start()
renderer = Renderer(scene)

plt.figure()
plt.axis('off')

while True:
    scene.update_for_tick(1/180.)
    renderer.sync_bodies()
    color = renderer.render_to_texture_of_color()
    time.sleep(1/180.)