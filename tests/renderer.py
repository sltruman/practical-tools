from math import pi, sin, cos
from direct.task import Task
from direct.showbase.ShowBase import ShowBase
from direct.filter.CommonFilters import CommonFilters
from direct.filter.FilterManager import FilterManager
from panda3d.core import NodePath

class Panda3DRenderer(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        self.scene = self.loader.loadModel("../practistyle\data\environment\japanese\japanese.dae")
        self.scene.reparentTo(self.render)
        self.scene.setScale(1, 1, 1)
        self.scene.setPos(0, 0, 0)
        self.taskMgr.add(self.spinCameraTask, "SpinCameraTask")

        self.buffer = self.win.makeTextureBuffer("hello", 256, 256)
        self.altCam = self.makeCamera(self.buffer)
        self.altCam.reparentTo(self.render)
        self.altCam.setPos(0, -10, 0)
        
    def spinCameraTask(self, task):
        angleDegrees = task.time * 6.0
        angleRadians = angleDegrees * (pi / 180.0)
        self.camera.setPos(20 * sin(angleRadians), -20 * cos(angleRadians), 3)
        self.camera.setHpr(angleDegrees, 0, 0)

        # texture = self.buffer.getTexture()
        # print(texture.getRamImageAs('RGBA'))

        return Task.cont

app = Panda3DRenderer()
app.run()
