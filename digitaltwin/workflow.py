from .scene import Scene
import pybullet as p
import math

class Workflow():
    def __init__(self,scene: Scene):
        self.scene = scene
        pass
    
    def node_kinds():
        return 0

    def start(self):
        self.scene.start()
        pass

    def stop(self):
        self.scene.stop()
        pass