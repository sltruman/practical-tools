import pybullet as p
import math
import numpy as np
import os
import traceback as tb
from .active_obj import ActiveObject

class Editor:
    def __init__(self,scene):
        self.scene = scene
        pass
        
    def ray(self,x,y):
        try:
            width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera()
            mouseX,mouseY = x*width,y*height

            camPos = camTarget - dist * np.array(camForward)

            farPlane = 10000
            rayForward = camTarget - camPos

            invLen = farPlane / dist
            rayForward = invLen * rayForward
            rayFrom = camPos

            oneOverWidth = float(1) / float(width)
            oneOverHeight = float(1) / float(height)

            horizon = np.array(horizon)
            vertical = np.array(vertical)

            dHor = horizon * oneOverWidth
            dVer = vertical * oneOverHeight

            rayToCenter = rayFrom + rayForward
            rayTo = rayToCenter - 0.5 * horizon + 0.5 * vertical + float(mouseX) * dHor - float(mouseY) * dVer
        except:
            tb.print_exc()
            return dict(name='',id=-1,pos=[0.,0.,0.])
        
        rayInfo = p.rayTest(rayFrom, rayTo)
        
        if not rayInfo: return dict(name='',id=-1,pos=[0.,0.,0.])
        id,linkindex,fraction,pos,norm = rayInfo[0]
        if id not in self.scene.active_objs: return dict(name='',id=-1,pos=pos)
        return dict(name=self.scene.active_objs[id].name,id=id,pos=pos)
        
    def add(self,name,active_obj):
        self.scene.active_objs[name] = active_obj
        return active_obj.properties()

    def remove(self,name):
        active_obj = self.scene.active_objs[name]
        del self.scene.active_objs[active_obj.id]
        return active_obj