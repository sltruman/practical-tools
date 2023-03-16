from .scene import Scene
import pybullet as p
import math

class Editor:
    def __init__(self,scene : Scene):
        self.scene = scene
        pass
        
    def ray(self,x,y):
        try:
            width, height, viewMat, projMat, cameraUp, camForward, horizon, vertical, _, _, dist, camTarget = p.getDebugVisualizerCamera()
            mouseX,mouseY = x*width,y*height

            camPos = [
                camTarget[0] - dist * camForward[0], 
                camTarget[1] - dist * camForward[1],
                camTarget[2] - dist * camForward[2]
            ]
            
            farPlane = 10000
            rayForward = [
                (camTarget[0] - camPos[0]), 
                (camTarget[1] - camPos[1]), 
                (camTarget[2] - camPos[2])]

            invLen = farPlane * 1. / (math.sqrt(rayForward[0] * rayForward[0] + rayForward[1] * rayForward[1] + rayForward[2] * rayForward[2]))
            rayForward = [
                invLen * rayForward[0], 
                invLen * rayForward[1], 
                invLen * rayForward[2]]
            
            rayFrom = camPos
            oneOverWidth = float(1) / float(width)
            oneOverHeight = float(1) / float(height)
            dHor = [horizon[0] * oneOverWidth, horizon[1] * oneOverWidth, horizon[2] * oneOverWidth]
            dVer = [vertical[0] * oneOverHeight, vertical[1] * oneOverHeight, vertical[2] * oneOverHeight]
            
            rayToCenter = [
                rayFrom[0] + rayForward[0], 
                rayFrom[1] + rayForward[1], 
                rayFrom[2] + rayForward[2]]
            
            rayTo = [
                rayFrom[0] + rayForward[0] - 0.5 * horizon[0] + 0.5 * vertical[0] + float(mouseX) * dHor[0] - float(mouseY) * dVer[0], 
                rayFrom[1] + rayForward[1] - 0.5 * horizon[1] + 0.5 * vertical[1] + float(mouseX) * dHor[1] - float(mouseY) * dVer[1], 
                rayFrom[2] + rayForward[2] - 0.5 * horizon[2] + 0.5 * vertical[2] + float(mouseX) * dHor[2] - float(mouseY) * dVer[2]]
        except:
            return dict(name='',id=-1,pos=[0.,0.,0.])
            
        rayInfo = p.rayTest(rayFrom, rayTo)
        p.addUserDebugLine(rayFrom,rayTo,[1,0,0],1,lifeTime=10)
        
        if not rayInfo: return dict(name='',id=-1,pos=[0.,0.,0.])
        id,linkindex,fraction,pos,norm = rayInfo[0]
        if id not in self.scene.active_objs: return dict(name='',id=-1,pos=pos)
        return dict(name=self.scene.active_objs[id].name,id=id,pos=pos)
        
    def move(self,name,pos):
        self.scene.active_objs_by_name[name].properties()
        pass

    def select(self,name)->dict:
        return self.scene.active_objs_by_name[name].properties()

    def add(self):
        pass
    
    def remove(self):
        pass

    def save(self):
        pass

