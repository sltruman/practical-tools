import pybullet as p
import numpy as np
from scipy.spatial.transform import Rotation
import multiprocessing as mp

class Render:
    def __init__(self,scene):
        self.p = mp.Process(target=self.run,args=())
        self.p.start()
        pass

    def __del__(self):
        pass

    def run(self):
        from math import pi, sin, cos
        from direct.task import Task
        from direct.showbase.ShowBase import ShowBase
        from direct.filter.CommonFilters import CommonFilters
        from direct.filter.FilterManager import FilterManager
        from panda3d.core import NodePath

        class App(ShowBase):
            def __init__(self):
                ShowBase.__init__(self)
                self.scene = self.loader.loadModel("models/environment")
                self.scene.reparentTo(self.render)
                self.scene.setScale(0.25, 0.25, 0.25)
                self.scene.setPos(-8, 42, 0)
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
                texture = self.buffer.getTexture()
                print(texture.getRamImageAs('RGBA'))

                return Task.cont

        app = App()
        app.run()
        pass

    def rtt(self):
        pass

    def sync_bodies(self):
        pass
    
    def draw_points(self):
        pass

    def draw_lines(self):
        pass
    