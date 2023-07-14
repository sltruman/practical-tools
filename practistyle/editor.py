import pybullet as p
import math
import numpy as np
import os
import traceback as tb
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
        
    def move(self,name,pos):
        self.scene.active_objs_by_name[name].properties()
        pass

    def select(self,name):
        return self.scene.active_objs_by_name[name].properties()

    def add(self,kind,base,pos,rot,extra_params={}):
        object_info = {
            "kind": kind,
            "base":base,
            "pos":pos,
            "rot":rot,
            "name":kind.lower()
        }

        object_info.update(extra_params)

        try:
            import practistyle
            active_obj = eval(f'practistyle.{kind}(self.scene,**object_info)')
        except:
            return {}
        
        self.scene.active_objs[active_obj.id] = active_obj
        
        name = active_obj.name
        i = 1
        while name in self.scene.active_objs_by_name:
            name = active_obj.name + str(i)
            i+=1
        
        active_obj.name = name
        self.scene.active_objs_by_name[active_obj.name] = active_obj
        return active_obj.properties()

    def remove(self,name):
        active_obj = self.scene.active_objs_by_name[name]
        del self.scene.active_objs[active_obj.id]
        del self.scene.active_objs_by_name[name]
        active_obj.remove()

    def rename(self,name,new_name):
        active_obj = self.scene.active_objs_by_name[name]
        del self.scene.active_objs_by_name[name]
        active_obj.name = new_name
        self.scene.active_objs_by_name[new_name] = active_obj

