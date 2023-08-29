from math import pi, sin, cos

from direct.showbase.ShowBase import ShowBase
from direct.task import Task
class MyApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        # Load the environment model.
        self.scene = self.loader.loadModel("models/environment")
        # Reparent the model to render.
        self.scene.reparentTo(self.render)

        # Apply scale and position transforms on the model.
        self.scene.setScale(0.25, 0.25, 0.25)
        self.scene.setPos(-8, 42, -1)

        obj = self.loader.loadModel("practistyle/data/pybullet_objects/plane.obj")
        # Reparent the model to render.
        obj.reparentTo(self.render)

        # Add the spinCameraTask procedure to the task manager.
        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")
        self.p = -45
        
        self.cam.setPos(0,0,200)
        self.cam.setHpr(0,self.p,0)
        


    # Define a procedure to move the camera.
    def spinCameraTask(self, task):
        self.p+=0.1
        self.cam.setHpr(0,self.p,0)
        return Task.cont


app = MyApp()
app.run()